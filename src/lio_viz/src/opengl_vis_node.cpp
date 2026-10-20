// opengl_vis_node.cpp
//
// Created by chuchu on 12/12/25.
//

#include "opengl_vis_node.h"
#include "../include/opengl_vis_node.h"



#include <iostream>
#include <sys/stat.h>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>
#include <GLFW/glfw3.h>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <yaml-cpp/yaml.h>
#include "DataType.h"

using namespace std;

static GLuint compile_shader(GLenum shader_type,
                            const char* shader_source,
                            const char* name,
                            const rclcpp::Logger& logger){
    GLuint shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, &shader_source, nullptr);
    glCompileShader(shader);
    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if(!success){
        GLint log_len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_len);
        std::string log(log_len, '\0');
        glGetShaderInfoLog(shader, log_len, nullptr, &log[0]);
        RCLCPP_ERROR(logger, "Shader compile error (%s): %s", name, log.c_str());
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

static GLuint link_program(GLuint vs, GLuint fs, const rclcpp::Logger &logger) {
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    GLint success = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &success);
    if (!success) {
        GLint log_len = 0;
        glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &log_len);
        std::string log(log_len, '\0');
        glGetProgramInfoLog(prog, log_len, nullptr, &log[0]);
        RCLCPP_ERROR(logger, "Program link error: %s", log.c_str());
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}
OpenGLPointCloudNode::OpenGLPointCloudNode()
    : rclcpp::Node("opengl_pointcloud_node"),
      window_(nullptr),
      zoom_(5.0f)
{
    recording_directory_ = declare_parameter<std::string>("recording_directory", "output/recordings");
    record_on_start_ = declare_parameter<bool>("record_on_start", false);
    recording_fps_ = declare_parameter<int>("recording_fps", 30);
    if (recording_fps_ < 1 || recording_fps_ > 120)
        throw std::invalid_argument("recording_fps must be between 1 and 120");
    camera_mode_ = declare_parameter<std::string>("camera_mode", "orbit");
    driving_distance_ = static_cast<float>(declare_parameter<double>("driving_distance", 6.0));
    driving_height_ = static_cast<float>(declare_parameter<double>("driving_height", 2.5));
    driving_lookahead_ = static_cast<float>(declare_parameter<double>("driving_lookahead", 12.0));
    if ((camera_mode_ != "orbit" && camera_mode_ != "driving") ||
        !std::isfinite(driving_distance_) || driving_distance_ < 0 ||
        !std::isfinite(driving_height_) || !std::isfinite(driving_lookahead_) || driving_lookahead_ <= 0)
        throw std::invalid_argument("Invalid driving camera configuration");
    standalone_cloud_path_ = declare_parameter<std::string>("cloud_path", "");
    non_loop_brightness_ = static_cast<float>(declare_parameter<double>("non_loop_brightness", 0.25));
    if (!std::isfinite(non_loop_brightness_) || non_loop_brightness_ < 0 || non_loop_brightness_ > 1)
        throw std::invalid_argument("non_loop_brightness must be between 0 and 1");
    color_mode_ = declare_parameter<std::string>("color_mode", "height_intensity");
    color_ground_z_ = static_cast<float>(declare_parameter<double>("color_ground_z", -1.7));
    if ((color_mode_ != "height_intensity" && color_mode_ != "intensity") || !std::isfinite(color_ground_z_))
        throw std::invalid_argument("Invalid color mode or road reference height");
    intensity_min_ = static_cast<float>(declare_parameter<double>("intensity_min", 0.0));
    intensity_max_ = static_cast<float>(declare_parameter<double>("intensity_max", 1.0));
    intensity_equalize_ = declare_parameter<bool>("intensity_equalize", true);
    for (size_t i = 0; i < intensity_cdf_.size(); ++i)
        intensity_cdf_[i] = static_cast<float>(i) / 255.0f;
    point_size_ = static_cast<float>(declare_parameter<double>("point_size", 1.5));
    current_frame_override_ = get_node_parameters_interface()->get_parameter_overrides().count("show_current_frame") != 0;
    show_current_frame_ = declare_parameter<bool>("show_current_frame", true);
    accumulated_map_override_ = get_node_parameters_interface()->get_parameter_overrides().count("show_accumulated_map") != 0;
    show_accumulated_map_ = declare_parameter<bool>("show_accumulated_map", false);
    hide_ground_override_ = get_node_parameters_interface()->get_parameter_overrides().count("hide_ground") != 0;
    hide_ground_ = declare_parameter<bool>("hide_ground", standalone_cloud_path_.empty() && camera_mode_ != "driving");
    ground_z_ = static_cast<float>(declare_parameter<double>("ground_z", 0.0));
    if (!std::isfinite(ground_z_))
        throw std::invalid_argument("Ground height must be finite");
    display_voxel_size_ = static_cast<float>(declare_parameter<double>("display_voxel_size", camera_mode_ == "driving" ? 0.1 : 0.35));
    local_voxel_size_ = static_cast<float>(declare_parameter<double>("local_voxel_size", camera_mode_ == "driving" ? 0.0 : 0.2));
    if (!std::isfinite(local_voxel_size_) || local_voxel_size_ < 0)
        throw std::invalid_argument("Local display voxel size must be finite and nonnegative");
    if (!std::isfinite(display_voxel_size_) || display_voxel_size_ < 0)
        throw std::invalid_argument("Display voxel size must be finite and nonnegative");
    if (!std::isfinite(intensity_min_) || intensity_min_ < 0 ||
        !std::isfinite(intensity_max_) || intensity_max_ <= intensity_min_ ||
        !std::isfinite(point_size_) || point_size_ <= 0)
        throw std::invalid_argument("Invalid intensity range or point size");
    if (!standalone_cloud_path_.empty()) {
        show_current_frame_ = true;
        show_accumulated_map_ = false;
        std::vector<PointVertex> points;
        if (!load_point_cloud_local(standalone_cloud_path_, points))
            throw std::runtime_error("Cannot load standalone cloud: " + standalone_cloud_path_);
        if (intensity_equalize_) loadIntensityDistribution(standalone_cloud_path_, true);
        for (const auto& point : points) {
            if (hide_ground_ && point.p_[2] <= ground_z_) continue;
            current_frame_points_.push_back(point);
            update_scene_bounds(glm::vec3(point.p_[0], point.p_[1], point.p_[2]));
        }
        current_frame_dirty_ = true;
        auto_fit_pending_ = true;
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "Loaded standalone cloud: %s (%zu displayed points)",
                    standalone_cloud_path_.c_str(), current_frame_points_.size());
    }
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "pcd_path",
        10,
        std::bind(&OpenGLPointCloudNode::on_pcd_path, this, std::placeholders::_1)
    );
    frame_sub_ = this->create_subscription<lio_msgs::msg::FrameData>(
     "frame_data",
     100,
     std::bind(&OpenGLPointCloudNode::on_key_frame_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Visualization node started. Waiting for pcd_path messages...");
    point_picker_.clear();

    loop_check_sub_ =
    this->create_subscription<std_msgs::msg::UInt64>(
        "/loop_closure/check_result",
        1,
        std::bind(
            &OpenGLPointCloudNode::onLoopCheckResult,
            this,
            std::placeholders::_1));

    viewer_ready_pub_ = create_publisher<std_msgs::msg::UInt64>("/loop_closure/viewer_ready", 1);

    viewer_loaded_pub_ = create_publisher<std_msgs::msg::UInt64>("/loop_closure/viewer_loaded", 1);

    loop_resume_pub_ =
    this->create_publisher<std_msgs::msg::UInt64>(
        "/loop_closure/resume", 1);
}

OpenGLPointCloudNode::~OpenGLPointCloudNode() {
    try { recorder_.close(); } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Video finalization failed: %s", e.what());
    }
    if (current_frame_vbo_) glDeleteBuffers(1, &current_frame_vbo_);
    if (current_frame_vao_) glDeleteVertexArrays(1, &current_frame_vao_);
    if (vbo_1_) glDeleteBuffers(1, &vbo_1_);
    if (vbo_2_) glDeleteBuffers(1, &vbo_2_);
    if (vao_1_) glDeleteVertexArrays(1, &vao_1_);
    if (vao_2_) glDeleteVertexArrays(1, &vao_2_);

    if (route_vbo_1_) glDeleteBuffers(1, &route_vbo_1_);
    if (route_vbo_2_) glDeleteBuffers(1, &route_vbo_2_);
    if (route_vao_1_) glDeleteVertexArrays(1, &route_vao_1_);
    if (route_vao_2_) glDeleteVertexArrays(1, &route_vao_2_);


  if(shader_program_){
    glDeleteProgram(shader_program_);
  }
    if (route_shader_program_) {
        glDeleteProgram(route_shader_program_);
    }

  if(window_){
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
  point_picker_.clear();
}

bool OpenGLPointCloudNode::init_window() {
    if (!glfwInit()) {
        RCLCPP_ERROR(this->get_logger(), "GLFW init failed");
        return false;
    }

    // Request OpenGL 3.3 core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(2000,2000 , "Point Cloud Viz", nullptr, nullptr);
    if (!window_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create window");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);

    // Load OpenGL functions with GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GLAD");
        return false;
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GLSL Version: "  << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    std::cout << "Renderer: "      << glGetString(GL_RENDERER) << std::endl;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);  // allow gl_PointSize in shaders

    // NEW: attach this object to the window, and set scroll callback
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, &OpenGLPointCloudNode::scroll_callback);
    glfwSetMouseButtonCallback(window_, &OpenGLPointCloudNode::mouse_button_callback);
    glfwSetCursorPosCallback(window_, &OpenGLPointCloudNode::cursor_pos_callback);
    glfwSetKeyCallback( window_, &OpenGLPointCloudNode::key_callback);
    if (!init_gl_resources()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init GL resources");
        return false;
    }
    return true;
}

bool OpenGLPointCloudNode::init_gl_resources() {
/*-----------------------Points Shader--------------------*/
    const char* vs_src = R"(
  #version 330 core
  layout (location = 0) in vec3 a_position;
  layout (location = 1) in float a_intensity;
  layout (location = 2) in float a_loop_role;
  layout (location = 3) in float a_height;

  uniform mat4 u_mvp;
  out float v_intensity;
  out float v_loop_role;
  out float v_height;
  uniform float u_point_size;
  void main(){
    gl_Position = u_mvp * vec4(a_position, 1.0);
    gl_PointSize = u_point_size;
    v_intensity = a_intensity;
    v_loop_role = a_loop_role;
    v_height = a_height;
 }
)";

    const char* fs_src = R"(
 #version 330 core

 in float v_intensity;
 in float v_loop_role;
 in float v_height;
 uniform bool u_height_color;
 uniform bool u_dim_non_loop;
 uniform float u_non_loop_brightness;
 uniform vec3 u_color;
 uniform float u_intensity_min;
 uniform float u_intensity_max;
 uniform bool u_equalize;
 uniform bool u_current_frame;
 uniform float u_cdf[256];
 out vec4 FragColor;

 // Sequential green palette with brighter high-intensity returns.
 vec3 intensityColor(float t) {
   // Current scan: orange -> amber -> cream against the green map.
   if (u_current_frame) {
     if (t < 0.5) return mix(vec3(0.85, 0.18, 0.03), vec3(1.0, 0.62, 0.08), t * 2.0);
     return mix(vec3(1.0, 0.62, 0.08), vec3(1.0, 0.97, 0.78), (t - 0.5) * 2.0);
   }
   vec3 c0 = vec3(0.06, 0.30, 0.18);
   vec3 c1 = vec3(0.10, 0.52, 0.28);
   vec3 c2 = vec3(0.20, 0.73, 0.37);
   vec3 c3 = vec3(0.48, 0.88, 0.46);
   vec3 c4 = vec3(0.84, 0.98, 0.66);
   if (t < 0.25) return mix(c0, c1, t * 4.0);
   if (t < 0.50) return mix(c1, c2, (t - 0.25) * 4.0);
   if (t < 0.75) return mix(c2, c3, (t - 0.50) * 4.0);
   return mix(c3, c4, (t - 0.75) * 4.0);
 }
 vec3 heightColor(float h) {
   vec3 road = vec3(0.58, 0.60, 0.63);
   vec3 low = vec3(0.24, 0.53, 0.90);
   vec3 middle = vec3(0.18, 0.76, 0.76);
   vec3 high = vec3(0.96, 0.67, 0.27);
   if (h <= 0.25) return road;
   if (h < 1.0) return mix(road, low, smoothstep(0.25, 1.0, h));
   if (h < 2.5) return mix(low, middle, (h - 1.0) / 1.5);
   return mix(middle, high, clamp((h - 2.5) / 3.0, 0.0, 1.0));
 }
 void main() {
   vec3 color = u_height_color ? heightColor(v_height) : u_color;
   float brightness = 0.85;
   if (v_intensity >= 0.0) {
     float t = clamp((v_intensity - u_intensity_min) /
                     (u_intensity_max - u_intensity_min), 0.0, 1.0);
     if (u_equalize) {
       float bin = t * 255.0;
       int lo = int(floor(bin));
       int hi = min(lo + 1, 255);
       t = mix(u_cdf[lo], u_cdf[hi], fract(bin));
     }
     // 256 color levels, indexed 0..255.
     t = floor(t * 255.0 + 0.5) / 255.0;
     if (u_height_color) brightness = 0.55 + 0.45 * sqrt(t);
     else color = intensityColor(t);
   }
   if (u_height_color) {
     color *= brightness;
     if (u_current_frame) color = min(color * 1.08, vec3(1.0));
   }
   if (u_dim_non_loop) {
     if (v_loop_role > 0.5 && v_loop_role < 1.5) color = vec3(1.0, 0.08, 0.06);
     else if (v_loop_role >= 1.5 && v_loop_role < 2.5) color = vec3(0.05, 1.0, 0.20);
     else color = vec3(u_non_loop_brightness * (0.65 + 0.35 * brightness));
   } else {
     if (v_loop_role > 0.5 && v_loop_role < 1.5) color = vec3(1.0, 0.25, 0.15);
     else if (v_loop_role >= 1.5 && v_loop_role < 2.5) color = vec3(0.1, 0.9, 1.0);
   }
   FragColor = vec4(color, 1.0);
 }
)";

auto logger = this->get_logger();

//commpile shader
GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src, "vertex", logger);
if(!vs){
return false;
}
GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src, "fragment", logger);
if(!fs){
  glDeleteShader(vs);
  return false;
}

//link shader progam
shader_program_ = link_program(vs, fs, logger);
glDeleteShader(vs);
glDeleteShader(fs);
if(!shader_program_) {
    return false;
}

/*-----------------------Route Shader--------------------*/
    const char* route_vs_src = R"(#version 330 core

layout(location = 0) in vec3 a_position;

uniform mat4 u_mvp;

void main() {
    gl_Position = u_mvp * vec4(a_position, 1.0);
})";

const char* route_fs_src = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 u_color;

void main() {
    FragColor = vec4(u_color, 1.0);
}
)";
    //commpile shader
    GLuint route_vs = compile_shader(GL_VERTEX_SHADER, route_vs_src, "r_vertex", logger);
    if(!route_vs){
        return false;
    }
    GLuint route_fs = compile_shader(GL_FRAGMENT_SHADER, route_fs_src, "r_fragment", logger);
    if(!route_fs){
        glDeleteShader(route_vs);
        return false;
    }

    //link shader progamname
    route_shader_program_ = link_program(route_vs, route_fs, logger);
    glDeleteShader(route_vs);
    glDeleteShader(route_fs);
    if(!route_shader_program_) {
        return false;
    }

//-----------------setup points vbo & vao---------------------------//
setupPointBuffers(vao_1_, vbo_1_);
setupPointBuffers(vao_2_, vbo_2_);
setupPointBuffers(current_frame_vao_, current_frame_vbo_);


//------------------------------setup route vbo & vao-------------------------------//
setupRouteBuffers(route_vao_1_, route_vbo_1_);
setupRouteBuffers(route_vao_2_, route_vbo_2_);

vbo_ready_1_ = false;
vbo_ready_2_ = false;

route_vbo_ready_1_ = false;
route_vbo_ready_2_ = false;



num_points_gpu_1_ = 0;
num_points_gpu_2_ = 0;

num_route_points_1_ = 0;
num_route_points_2_ = 0;


return true;
}

void OpenGLPointCloudNode::upload_points_to_gpu(const std::vector<PointVertex>& pts,
    GLuint vbo,
    bool& ready,
    size_t& count){

  if(!vbo)return;

  //bind ptr data
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, pts.size() * sizeof(PointVertex), nullptr, GL_DYNAMIC_DRAW);

    glBufferSubData(GL_ARRAY_BUFFER, 0, pts.size() * sizeof(PointVertex), pts.data());
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  count = pts.size();
  ready = (count > 0);

  RCLCPP_INFO(this->get_logger(), "Uploaded %zu points to GPU", count);
}

void OpenGLPointCloudNode::upload_route_to_gpu(const std::vector<glm::vec3>&route_pts, GLuint route_vbo, bool& ready, size_t& count) {
    glBindBuffer(GL_ARRAY_BUFFER, route_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 route_pts.size() * sizeof(glm::vec3),
                 route_pts.data(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    count = route_pts.size();
    ready= (count > 0);
}

void OpenGLPointCloudNode::draw_points_layer(const glm::mat4 &mvp, GLuint vao, bool vbo_ready, size_t num_points, const glm::vec3& color, bool current_frame) {
    if (!vbo_ready || num_points ==0) {
        return;
    }
    glUseProgram (shader_program_);
    glUniform1f(glGetUniformLocation(shader_program_, "u_intensity_min"), intensity_min_);
    glUniform1f(glGetUniformLocation(shader_program_, "u_intensity_max"), intensity_max_);
    glUniform1f(glGetUniformLocation(shader_program_, "u_point_size"), current_frame ? point_size_ + (color_mode_ == "height_intensity" ? 0.3f : 1.0f) : point_size_);
    glUniform1i(glGetUniformLocation(shader_program_, "u_current_frame"), current_frame);
    glUniform1i(glGetUniformLocation(shader_program_, "u_height_color"), color_mode_ == "height_intensity");
    glUniform1i(glGetUniformLocation(shader_program_, "u_dim_non_loop"), show_loop_pairs_ && dim_non_loop_);
    glUniform1f(glGetUniformLocation(shader_program_, "u_non_loop_brightness"), non_loop_brightness_);
    glUniform1i(glGetUniformLocation(shader_program_, "u_equalize"), intensity_equalize_);
    glUniform1fv(glGetUniformLocation(shader_program_, "u_cdf[0]"), 256, intensity_cdf_.data());

    //set mvp
    GLuint loc_mvp  = glGetUniformLocation(shader_program_, "u_mvp");
    glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, glm::value_ptr(mvp));

    GLint loc_color = glGetUniformLocation(shader_program_, "u_color");
    glUniform3f(
        loc_color,
        color.x,
        color.y,
        color.z);

    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(num_points));
    glBindVertexArray(0);
    glUseProgram(0);
}

void OpenGLPointCloudNode::draw_route_layer(const glm::mat4& mvp, GLuint vao, bool vbo_ready, size_t num_points_route, const glm::vec3& color) {
    if (!vbo_ready || num_points_route == 0) {
        return;
    }
    glUseProgram(route_shader_program_);
    // MVP
    GLint loc_mvp = glGetUniformLocation(route_shader_program_, "u_mvp");
    glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, glm::value_ptr(mvp));

    GLint loc_color = glGetUniformLocation(route_shader_program_, "u_color");
    glUniform3f(loc_color, color.x, color.y, color.z);

    glBindVertexArray(vao);
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(num_points_route));
    glBindVertexArray(0);
    glUseProgram(0);
}

void OpenGLPointCloudNode::on_pcd_path(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex>lock(mutex_);
    std::cout<<"vis node recive: " << msg->data<<std::endl;
    if (msg->data != " ") {
        pending_paths_.push(msg->data);
    }
}

void OpenGLPointCloudNode::on_key_frame_callback(const lio_msgs::msg::FrameData::SharedPtr msg) {
   if (!standalone_cloud_path_.empty()) return;
   if (msg->refresh_id != active_refresh_id_) return;
   if(!initialized_) {
      std::string init_path = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/Config/config.yaml";
       if (loadVisNodeConfig(init_path)) {
           initialized_ = true;
       }
   }
    auto printPose = [&](const char* name, const glm::mat4& T) {
        double yaw = std::atan2(T[0][1], T[0][0]);
        std::cout << name
                  << " t = " << T[3][0] << " " << T[3][1] << " " << T[3][2]
                  << " yaw = " << yaw
                  << std::endl;
   };
   PendingFrame pf;
   pf.refresh_id_ = msg->refresh_id;
   pf.refresh_last_ = msg->refresh_last;
   pf.frame_id_ = msg->frame_id;
   pf.submap_id_ = msg->submap_id;
   pf.cloud_path_ = msg->cloud_path;
   pf.loop_role_ = msg->loop_role;
   pf.lidar_pose_ = poseMsgToGlm(msg->lidar_pose);
   pf.rtk_pose_ = poseMsgToGlm(msg->rtk_pose);
   pf.lidar_pose_neu_ = poseMsgToGlm(msg->lidar_pose_neu);
   pf.fst_optimization_pose_= poseMsgToGlm((msg->fst_optimization_pose));
   pf.scd_optimization_pose_= poseMsgToGlm(msg->scd_optimization_pose);
   pf.loop_optimization_pose_ = poseMsgToGlm(msg->loop_optimization_pose);


    //printPose("lidar", pf.lidar_pose_);
    printPose("lidar_neu", pf.loop_optimization_pose_);


   // pf.has_lidar_pose_ = msg->has_lidar_pose;
   // pf.has_rtk_pose_ = msg->has_rtk_pose;
   // pf.has_lidar_pose_neu_ = msg->has_lidar_pose_neu;

   std::lock_guard<std::mutex>lock(pending_mutex_);
   pending_frames_.push_back(std::move(pf));
}

void OpenGLPointCloudNode::consume_pending_frames() {
  std::deque<PendingFrame> local;
  {
     std::lock_guard<std::mutex> lock(pending_mutex_);
     local.swap(pending_frames_);
  }

  for(auto& pf :  local) {
    if (pf.refresh_id_ != active_refresh_id_) continue;
    const PoseType camera_pose_type = layer2_.draw_map_ ? layer2_.map_pose_type_ :
        layer1_.draw_map_ ? layer1_.map_pose_type_ :
        layer2_.draw_route_ ? layer2_.route_pose_type_ : layer1_.route_pose_type_;
    driving_pose_ready_ = selectPose(pf, camera_pose_type, driving_pose_);
    std::cout<<"frame id: "<<pf.frame_id_<<std::endl;
    std::vector<PointVertex>local_points_raw;
    std::vector<PointVertex> local_points;
    if (show_current_frame_ || layer1_.draw_map_ || layer2_.draw_map_) {
       if (!load_point_cloud_local(pf.cloud_path_, local_points_raw)) {
           RCLCPP_ERROR(get_logger(), "Cannot load snapshot cloud: %s", pf.cloud_path_.c_str());
       }
       if (local_voxel_size_ > 0) local_points = voxelDownsampleLocal(local_points_raw, local_voxel_size_);
       else local_points = std::move(local_points_raw);
    }
    if (show_current_frame_) {
        current_frame_points_.clear();
        if (!show_accumulated_map_) {
            map_min_x_ = map_min_y_ = map_min_z_ = std::numeric_limits<float>::max();
            map_max_x_ = map_max_y_ = map_max_z_ = std::numeric_limits<float>::lowest();
        }
        // Match the visible map's pose, preferring its optimized layer.
        PoseType type = layer2_.draw_map_ ? layer2_.map_pose_type_ : layer1_.map_pose_type_;
        glm::mat4 pose;
        if (selectPose(pf, type, pose)) {
            current_frame_points_.reserve(local_points.size());
            for (auto point : local_points) {
                glm::vec4 q = pose * glm::vec4(point.p_[0], point.p_[1], point.p_[2], 1.0f);
                if (hide_ground_ && q.z <= ground_z_) continue;
                point.p_[0] = q.x; point.p_[1] = q.y; point.p_[2] = q.z;
                point.loop_role_ = 0.0f;
                current_frame_points_.push_back(point);
                update_scene_bounds(glm::vec3(q));
            }
        }
        current_frame_dirty_ = true;
    }
    if (show_accumulated_map_) {
        append_frame_to_layer(pf, local_points, layer1_, map_points_1_, route_points_1_);
        append_frame_to_layer(pf, local_points, layer2_, map_points_2_, route_points_2_);
    }
      if (layer1_.draw_map_) {
          gpu_dirty_1_ = true;
      }

      if (layer2_.draw_map_) {
          gpu_dirty_2_ = true;
      }
      if (layer1_.draw_route_) {
          route_gpu_dirty_1_ = true;
      }

      if (layer2_.draw_route_) {
          route_gpu_dirty_2_ = true;
      }
    auto_fit_pending_ = true;
    if (pf.refresh_last_) {
        loop_inspection_mode_ = true;
        std_msgs::msg::UInt64 ready;
        ready.data = active_refresh_id_;
        viewer_loaded_pub_->publish(ready);
    }
  }
  point_picker_.build();
}

void OpenGLPointCloudNode::append_frame_to_layer(const PendingFrame &pf, const std::vector<PointVertex> &local_points,
    const RenderLayerConfig &config, std::vector<PointVertex> &map_points, std::vector<glm::vec3> &route_points) {
    if (config.draw_map_) {
       //convert local points to wc
       glm::mat4 T_map;

       if(selectPose(pf, config.map_pose_type_, T_map)) {
           size_t point_idx =0;
           for (const auto& p : local_points) {
               glm::vec4 q = T_map * glm::vec4(p.p_[0], p.p_[1], p.p_[2], 1.0f);
               if (hide_ground_ && q.z <= ground_z_) continue;
               PointVertex wp =p;
               wp.p_[0] = q.x;
               wp.p_[1] = q.y;
               wp.p_[2] = q.z;
               wp.loop_role_ = show_loop_pairs_ ? static_cast<float>(pf.loop_role_) : 0.0f;
               point_idx++;
               if(point_idx %3 == 0) {
                 //pick points
                 Vec3d w_pick_p = {q.x, q.y, q.z};
                 point_picker_.addPoint(w_pick_p, pf.frame_id_, pf.submap_id_, point_idx, config.id_);
               }
               if (display_voxel_size_ > 0) {
                   const float inv = 1.0f / display_voxel_size_;
                   PtrVoxelKey key{static_cast<int32_t>(std::floor(q.x * inv)),
                                   static_cast<int32_t>(std::floor(q.y * inv)),
                                   static_cast<int32_t>(std::floor(q.z * inv))};
                   // Keep both loop roles and context in separate voxel buckets;
                   // otherwise overlapping red/green clouds overwrite one another.
                   const size_t role = wp.loop_role_ > 1.5f && wp.loop_role_ < 2.5f ? 2 :
                                       wp.loop_role_ > 0.5f && wp.loop_role_ < 1.5f ? 1 : 0;
                   auto& voxel = display_voxels_[config.id_][role][key];
                   voxel.sx += q.x; voxel.sy += q.y; voxel.sz += q.z;
                   voxel.si += wp.intensity_; voxel.st += wp.height_; ++voxel.count;
                   if (voxel.render_idx_ < 0) {
                       voxel.render_idx_ = static_cast<int>(map_points.size());
                       map_points.push_back(wp);
                   }
                   auto& displayed = map_points[voxel.render_idx_];
                   displayed.p_[0] = static_cast<float>(voxel.sx / voxel.count);
                   displayed.p_[1] = static_cast<float>(voxel.sy / voxel.count);
                   displayed.p_[2] = static_cast<float>(voxel.sz / voxel.count);
                   displayed.intensity_ = static_cast<float>(voxel.si / voxel.count);
                   displayed.height_ = static_cast<float>(voxel.st / voxel.count);
                   if (wp.loop_role_ > 0) displayed.loop_role_ = wp.loop_role_;
               } else {
                   map_points.push_back(wp);
               }
               update_scene_bounds(glm::vec3(wp.p_[0], wp.p_[1], wp.p_[2]));
           }
       }
    }
    if (config.draw_route_) {
        glm::mat4 T_route;
        if (selectPose(pf, config.route_pose_type_, T_route)) {
            glm::vec3 route_p = getTranslation(T_route);
            if (route_points.empty()|| glm::length(route_points.back()-route_p)> 0.5f) {
              route_points.push_back(route_p);
              update_scene_bounds(route_p);
            }
        }
    }
}

bool OpenGLPointCloudNode::load_point_cloud_local(const std::string &path, std::vector<PointVertex> &out_points) {
    out_points.clear();

    auto cloud = std::make_shared<PointCloud>();
    bool has_intensity = false;
    if (!loadPointCloudFromFile(path, cloud, &has_intensity)) return false;
    out_points.reserve(cloud->size());
    for (const auto& point : *cloud) {
        PointVertex p;
        p.p_[0] = point.x; p.p_[1] = point.y; p.p_[2] = point.z;
        p.height_ = point.z - color_ground_z_;
        p.intensity_ = has_intensity ? std::max(0.0f, point.intensity) : -1.0f;
        out_points.push_back(p);
    }
    return true;
}

bool OpenGLPointCloudNode::selectPose(const PendingFrame &pf, PoseType pose, glm::mat4 &T) const {
switch (pose) {
    case PoseType::LIDAR:
    {
        if (!pf.has_lidar_pose_) {
            return false;
        }

        T = pf.lidar_pose_;
        return true;
    }

    case PoseType::RTK:
    {
        if (!pf.has_rtk_pose_) {
            return false;
        }

        T = pf.rtk_pose_;
        return true;
    }

    case PoseType::LIDAR_NEU:
    {
        if (!pf.has_lidar_pose_neu_) {
            return false;
        }

        T = pf.lidar_pose_neu_;
        return true;
    }
    case PoseType::FST_OPTIMIZATION: {
        if (!pf.has_fst_optimization_pose_) {
            return false;
        }
        T = pf.fst_optimization_pose_;
        return true;
    }

    case PoseType::SCD_OPTIMIZATION: {
        if (!pf.has_scd_optimization_pose_) {
            return false;
        }
        T = pf.scd_optimization_pose_;
        return true;
    }
    case PoseType::LOOP_OPTIMIZATION: {
        if (!pf.has_loop_optimization_pose_) {
            return false;
        }
        T = pf.loop_optimization_pose_;
        return true;
    }

}
    return false;
}

glm::vec3 OpenGLPointCloudNode::getTranslation(const glm::mat4 &T) const {
    return glm::vec3(
       T[3][0],
       T[3][1],
       T[3][2]
   );
}

bool OpenGLPointCloudNode::setupPointBuffers(GLuint &vao, GLuint &vbo) {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    if (!vao || !vbo) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create VAO/VBO");
        return false;
    }
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    //configure attribute layout
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PointVertex), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
      1,
      1,
      GL_FLOAT,
      GL_FALSE,
      sizeof(PointVertex),
      (void*)(3 * sizeof(float)) // offset
    );
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(PointVertex),
                          reinterpret_cast<void*>(offsetof(PointVertex, loop_role_)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(PointVertex),
                          reinterpret_cast<void*>(offsetof(PointVertex, height_)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    return true;
}

bool OpenGLPointCloudNode::setupRouteBuffers(GLuint &route_vao, GLuint &route_vbo) {
    glGenVertexArrays(1, &route_vao);
    glGenBuffers(1, &route_vbo);

    glBindVertexArray(route_vao);
    glBindBuffer(GL_ARRAY_BUFFER, route_vbo);
    glBufferData(GL_ARRAY_BUFFER, 0 , nullptr, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(glm::vec3),(void*)0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    return true;
}

bool OpenGLPointCloudNode::loadVisNodeConfig(const std::string &path) {
   YAML::Node cfg;
   try {
      cfg = YAML::LoadFile(path);
   }
   catch (const std::exception &e) {
       RCLCPP_ERROR(
           this->get_logger(),
           "Failed loading vis yaml %s : %s",
           path.c_str(),
           e.what());
       return false;
   }
   auto vis = cfg["vis"];
   if (!vis){
    RCLCPP_ERROR(
        this->get_logger(),
        "Missing vis section");

    return false;
   }
    if (!current_frame_override_ && standalone_cloud_path_.empty() && vis["show_current_frame"])
        show_current_frame_ = vis["show_current_frame"].as<bool>();
    if (!accumulated_map_override_ && standalone_cloud_path_.empty())
        show_accumulated_map_ = vis["show_accumulated_map"] && vis["show_accumulated_map"].as<bool>();
    if (intensity_equalize_ && cfg["backend"] && cfg["backend"]["in_kf_info_path"])
        loadIntensityDistribution(cfg["backend"]["in_kf_info_path"].as<std::string>());
    if (!hide_ground_override_ && camera_mode_ != "driving" && vis["hide_ground"])
        hide_ground_ = vis["hide_ground"].as<bool>();
    show_loop_pairs_ = vis["show_loop_pairs"] && vis["show_loop_pairs"].as<bool>();
    dim_non_loop_ = vis["dim_non_loop"] && vis["dim_non_loop"].as<bool>();
    layer1_.id_ = 0;
    loadLayerConfig(
     vis["layer1"],
     layer1_);

    layer2_.id_ = 1;
    loadLayerConfig(
        vis["layer2"],
        layer2_);

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded vis config");

    return true;
}

void OpenGLPointCloudNode::loadIntensityDistribution(const std::string& keyframe_path, bool single_cloud) {
    // Use the complete saved route, so the mapping remains stable during replay.
    std::array<uint64_t, 256> histogram {};
    std::vector<std::string> paths;
    if (single_cloud) paths.push_back(keyframe_path);
    else {
        std::ifstream frames(keyframe_path);
        std::string line;
        while (std::getline(frames, line)) {
            std::istringstream row(line);
            std::string id, submap, role, path;
            if (row >> id >> submap >> role >> path) paths.push_back(path);
        }
    }
    uint64_t total = 0;
    for (const auto& path : paths) {
        std::ifstream file(path + ".intensity", std::ios::binary);
        float value;
        while (file.read(reinterpret_cast<char*>(&value), sizeof(value))) {
            if (!std::isfinite(value) || value < 0) continue;
            float t = std::max(0.0f, std::min(1.0f,
                (value - intensity_min_) / (intensity_max_ - intensity_min_)));
            ++histogram[static_cast<size_t>(std::floor(t * 255.0f))];
            ++total;
        }
    }
    for (size_t i = 0; i < intensity_cdf_.size(); ++i)
        intensity_cdf_[i] = static_cast<float>(i) / 255.0f;
    if (!total) {
        RCLCPP_WARN(get_logger(), "No saved intensity distribution; using linear colors");
        return;
    }
    size_t first = 0, last = 255;
    while (!histogram[first]) ++first;
    while (!histogram[last]) --last;
    if (first == last) return; // Constant intensity has no contrast to recover.
    const double low = 0.5 * histogram[first];
    const double high = static_cast<double>(total) - 0.5 * histogram[last];
    uint64_t cumulative = 0;
    for (size_t i = 0; i < histogram.size(); ++i) {
        double midpoint = static_cast<double>(cumulative) + 0.5 * histogram[i];
        intensity_cdf_[i] = static_cast<float>(std::max(0.0, std::min(1.0,
            (midpoint - low) / (high - low))));
        cumulative += histogram[i];
    }
    RCLCPP_INFO(get_logger(), "Equalized %llu saved intensities across 256 color levels",
                static_cast<unsigned long long>(total));
}

void OpenGLPointCloudNode::loadLayerConfig(const YAML::Node &node, RenderLayerConfig &layer) {
    layer.draw_map_ =
        node["draw_map"]
        .as<bool>();

    layer.draw_route_ =
        node["draw_route"]
        .as<bool>();

    layer.map_pose_type_ =
        parsePoseType(
            node["map_pose"]
            .as<std::string>());

    layer.route_pose_type_ =
        parsePoseType(
            node["route_pose"]
            .as<std::string>());

    auto map_color =
        node["map_color"];

    layer.map_color_ =
        glm::vec3(
            map_color[0].as<float>(),
            map_color[1].as<float>(),
            map_color[2].as<float>());

    auto route_color =
        node["route_color"];

    layer.route_color_ =
        glm::vec3(
            route_color[0].as<float>(),
            route_color[1].as<float>(),
            route_color[2].as<float>());
}

PoseType OpenGLPointCloudNode::parsePoseType(const std::string &s) {
    if (s == "lidar")
        return PoseType::LIDAR;

    if (s == "rtk")
        return PoseType::RTK;

    if (s == "lidar_neu")
        return PoseType::LIDAR_NEU;

    if (s == "fst_optimization")
        return PoseType::FST_OPTIMIZATION;

    if (s== "scd_optimization")
        return PoseType::SCD_OPTIMIZATION;

    if (s == "loop_optimization")
        return PoseType::LOOP_OPTIMIZATION;

    RCLCPP_WARN(
        this->get_logger(),
        "Unknown pose type: %s, fallback to lidar",
        s.c_str());

    return PoseType::LIDAR;
}

void OpenGLPointCloudNode::merge_points(const std::vector<PointVertex> &new_points) {
   //Todo: add voxel down sampel and also remove overlap
   for (auto& p:  new_points) {
       map_points_.push_back(p);
       update_scene_bounds(glm::vec3(p.p_[0], p.p_[1], p.p_[2]));
   }
}

void OpenGLPointCloudNode::update_scene_bounds(const glm::vec3& p) {
    map_min_x_ = std::min(map_min_x_, p.x);
    map_min_y_ = std::min(map_min_y_, p.y);
    map_min_z_ = std::min(map_min_z_, p.z);

    map_max_x_ = std::max(map_max_x_, p.x);
    map_max_y_ = std::max(map_max_y_, p.y);
    map_max_z_ = std::max(map_max_z_, p.z);
}


void OpenGLPointCloudNode::render_frame(float t){
  if(!window_){
    return;
  }

  consume_pending_frames();
  update_scene_state();
  //std::cout<<"update rendering"<<std::endl;
  // events
  glfwPollEvents();

  int w, h;
  glfwGetFramebufferSize(window_, &w, &h);
  float aspect  = (float) w / float(h? h:1);

  glViewport(0, 0, w, h);
  glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::mat4 mvp = compute_mvp(aspect);
    if (show_accumulated_map_) {
    draw_points_layer(
       mvp,
       vao_1_,
       vbo_ready_1_,
       num_points_gpu_1_,
       layer1_.map_color_
       );
    cur_mvp_ = mvp;
    draw_points_layer(
        mvp,
        vao_2_,
        vbo_ready_2_,
        num_points_gpu_2_,
        layer2_.map_color_
        );
    }
    if (show_current_frame_) {
        glDepthFunc(GL_LEQUAL);
        draw_points_layer(mvp, current_frame_vao_, current_frame_ready_,
                          current_frame_count_, glm::vec3(1.0f, 0.65f, 0.1f), true);
        glDepthFunc(GL_LESS);
    }
    cur_mvp_ = mvp;
    if (show_accumulated_map_) {
    draw_route_layer(
        mvp,
        route_vao_1_,
        route_vbo_ready_1_,
        num_route_points_1_,
        layer1_.route_color_);

    draw_route_layer(
        mvp,
        route_vao_2_,
        route_vbo_ready_2_,
        num_route_points_2_,
        layer2_.route_color_
        );
    }
  if (record_on_start_ && (driving_pose_ready_ || !standalone_cloud_path_.empty())) {
      record_on_start_ = false;
      toggleRecording();
  }
  captureRecordingFrame(w, h);
  glfwSwapBuffers(window_);
}

void OpenGLPointCloudNode::toggleRecording() {
    try {
        if (recorder_.active()) {
            recorder_.close();
            RCLCPP_INFO(get_logger(), "Saved recording: %s", recording_path_.c_str());
            return;
        }
        // Create nested output directories without invoking a shell.
        std::string prefix;
        for (char c : recording_directory_ + "/") {
            prefix += c;
            if (c == '/' && prefix.size() > 1) ::mkdir(prefix.c_str(), 0755);
        }
        const auto now = std::chrono::system_clock::now();
        const auto stamp = std::chrono::system_clock::to_time_t(now);
        std::ostringstream name;
        name << recording_directory_ << "/optimized_map_" << std::put_time(std::localtime(&stamp), "%Y%m%d_%H%M%S")
             << "_" << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()%1000 << ".avi";
        recording_path_ = name.str();
        glfwGetFramebufferSize(window_, &recording_width_, &recording_height_);
        recorder_.open(recording_path_, recording_width_, recording_height_, recording_fps_);
        recording_start_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Recording %s; press R to stop", recording_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Recording failed: %s", e.what());
    }
}

void OpenGLPointCloudNode::captureRecordingFrame(int width, int height) {
    if (!recorder_.active()) return;
    try {
        if (width != recording_width_ || height != recording_height_) {
            recorder_.close();
            RCLCPP_WARN(get_logger(), "Window resized; saved recording. Press R to start another.");
            return;
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-recording_start_).count();
        const uint64_t wanted = static_cast<uint64_t>(seconds * recording_fps_) + 1;
        if (wanted <= recorder_.frames()) return;
        std::vector<unsigned char> rgb(static_cast<size_t>(width) * height * 3);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
        recorder_.append(rgb.data(), static_cast<int>(wanted-recorder_.frames()));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Recording stopped: %s", e.what());
        try { recorder_.close(); } catch (...) {}
    }
}

void OpenGLPointCloudNode::update_scene_state() {
    if (current_frame_dirty_) {
        upload_points_to_gpu(current_frame_points_, current_frame_vbo_,
                             current_frame_ready_, current_frame_count_);
        current_frame_dirty_ = false;
    }
    if (auto_fit_pending_ && (!map_points_1_.empty() || !map_points_2_.empty()|| !route_points_1_.empty() ||
     !route_points_2_.empty() || !current_frame_points_.empty())) {
        compute_view_params();
        auto_fit_pending_ = false;
     }

    if (gpu_dirty_1_) {
        upload_points_to_gpu(
            map_points_1_,
            vbo_1_,
            vbo_ready_1_,
            num_points_gpu_1_);

        gpu_dirty_1_ = false;
    }

    if (gpu_dirty_2_) {
        upload_points_to_gpu(
            map_points_2_,
            vbo_2_,
            vbo_ready_2_,
            num_points_gpu_2_);

        gpu_dirty_2_ = false;
    }

    if (route_gpu_dirty_1_) {
        upload_route_to_gpu(
            route_points_1_,
            route_vbo_1_,
            route_vbo_ready_1_,
            num_route_points_1_);

        route_gpu_dirty_1_ = false;
    }

    if (route_gpu_dirty_2_) {
        upload_route_to_gpu(
            route_points_2_,
            route_vbo_2_,
            route_vbo_ready_2_,
            num_route_points_2_);

        route_gpu_dirty_2_ = false;
    }
}

 glm::mat4 OpenGLPointCloudNode::compute_mvp(float aspect) const {
    /*Point (x,y,z)
        ↓
        model
    World space
        ↓ view
    Camera space0
        ↓ projection
    Clip space → screen
    */

    if (camera_mode_ == "driving" && driving_pose_ready_) {
        const glm::vec3 position(driving_pose_[3]);
        glm::vec3 forward(driving_pose_[0].x, driving_pose_[0].y, 0.0f);
        if (glm::length(forward) < 1e-6f) forward = glm::vec3(1, 0, 0);
        forward = glm::normalize(forward);
        const glm::vec3 eye = position - forward * driving_distance_ + glm::vec3(0, 0, driving_height_);
        const glm::vec3 target = position + forward * driving_lookahead_;
        return glm::perspective(glm::radians(65.0f), aspect, 0.1f, 2000.0f) *
               glm::lookAt(eye, target, glm::vec3(0, 0, 1));
    }
    float dist = base_distance_ * zoom_;

     // 2. Convert angles (VERY IMPORTANT: degrees → radians)
     float pitch = glm::radians(pitch_deg_);
     float yaw   = glm::radians(yaw_deg_);

     // 3. Compute camera offset using spherical coordinates
     glm::vec3 offset;
     offset.x = dist * std::cos(pitch) * std::sin(yaw);
     offset.y = dist * std::cos(pitch) * std::cos(yaw);
     offset.z = dist * std::sin(pitch);

    glm::vec3 cam_target = view_center_;
     glm::vec3 cam_pos = cam_target + offset;
     glm::vec3 cam_up = cam_up = glm::vec3(0.0f, 0.0f, 1.0f);
     // if (std::abs(std::abs(pitch_deg_) - 90.0f) < 0.1f) {
     //     // near top-down → use Y as up
     //     cam_up = glm::vec3(0.0f, 1.0f, 0.0f);
     // } else {
     //     // normal case → Z is up
     //     cam_up = glm::vec3(0.0f, 0.0f, 1.0f);
     // }

     // 7. Projection matrix (Perspective)
     float near_plane = 0.1f;
     float far_plane = std::max(
    1000.0f,
    dist + 10.0f * base_distance_);
     glm::mat4 proj = glm::perspective(
         glm::radians(60.0f),                 // FOV
         aspect,                              // width / height
         near_plane,                                // near plane
         far_plane      // far plane
     );

    glm::mat4 view = glm::lookAt(cam_pos, cam_target, cam_up);

    glm::mat4 model(1.0f);
    // Usually you do NOT rotate model again if camera yaw/pitch already controls view.
    // Keep this identity unless you really want object rotation.
    return proj * view * model;
}

void OpenGLPointCloudNode::compute_view_params() {
    float range_x = map_max_x_ - map_min_x_;
    float range_y = map_max_y_ - map_min_y_;
    float range_z = map_max_z_ - map_min_z_;
    float max_range = std::max(range_x, std::max(range_y, range_z));
    if (max_range < 1.0f) {
        max_range = 1.0f;
    }

    view_center_ = glm::vec3(
        0.5f * (map_min_x_ + map_max_x_),
        0.5f * (map_min_y_ + map_max_y_),
        0.5f * (map_min_z_ + map_max_z_)
    );

    base_distance_ = 1.5f * max_range + 1.0f;
}

std::vector<PointVertex> OpenGLPointCloudNode::voxelDownsampleLocal(const std::vector<PointVertex> &input,
    double voxel_size) {
    std::unordered_map<PtrVoxelKey, VoxelAccum, PtrVoxelHash> voxel_map;
    const float inv = 1.0f / static_cast<float>(voxel_size);
    for (const auto& p : input) {
       PtrVoxelKey key{
         static_cast<int32_t>(std::floor(p.p_[0] * inv)),
         static_cast<int32_t>(std::floor(p.p_[1] * inv)),
         static_cast<int32_t>(std::floor(p.p_[2] * inv))
       };
       auto& a = voxel_map[key];
       a.sx += p.p_[0];
       a.sy += p.p_[1];
       a.sz += p.p_[2];
       a.si += p.intensity_;
       a.st += p.height_;
       a.count += 1;
    }
    std::vector<PointVertex>output;
    output.reserve(voxel_map.size());
    for (auto&kv :voxel_map) {
        const auto& a = kv.second;
        const float inv_count = 1.0f / static_cast<float>(a.count);

        PointVertex p;
        p.p_[0] = static_cast<float>(a.sx * inv_count);
        p.p_[1] = static_cast<float>(a.sy * inv_count);
        p.p_[2] = static_cast<float>(a.sz * inv_count);
        p.intensity_ = static_cast<float>(a.si * inv_count);
        p.height_ = static_cast<float>(a.st * inv_count);

        output.push_back(p);
    }
    return output;
}


void OpenGLPointCloudNode::scroll_callback(GLFWwindow* window,
                                           double xoffset,
                                           double yoffset){
    (void)xoffset;  // unused

    // Recover the C++ object from the GLFW window
    auto* self = static_cast<OpenGLPointCloudNode*>(
        glfwGetWindowUserPointer(window));
    if (!self) return;

    if (yoffset > 0.0) {
        self->zoom_ *= 0.9f;
    } else if (yoffset < 0.0) {
        self->zoom_ *= 1.1f;
    }

    if (self->zoom_ < 0.1f) self->zoom_ = 0.1f;
    if (self->zoom_ > 20.0f) self->zoom_ = 20.0f;
}

void OpenGLPointCloudNode::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
(void)mods;
    (void)mods;
    auto* self =
        static_cast<OpenGLPointCloudNode*>(
            glfwGetWindowUserPointer(window));

    if (!self) {
        return;
    }

    if (action ==GLFW_PRESS) {
        glfwGetCursorPos(window, &self->last_x_, &self->last_y_);
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
           self->dragging_ = true;
           self->mouse_down_x_ =self->last_x_;
           self->mouse_down_y_ =self->last_y_;
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
           self->panning_ = true;
        }
    }
    else if (action == GLFW_RELEASE) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            self->dragging_ = false;

            double release_x;
            double release_y;

            glfwGetCursorPos(
                window,
                &release_x,
                &release_y);

            double dx = release_x - self->mouse_down_x_;
            double dy = release_y - self->mouse_down_y_;

            double move2 = dx * dx + dy * dy;

            if (move2 < 25.0) {
                self->handlePick(release_x, release_y);
            }
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            self->panning_ = false;
        }
    }
}

void OpenGLPointCloudNode::cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
  auto* self = static_cast<OpenGLPointCloudNode*>(glfwGetWindowUserPointer(window));
  if (!self)return;

   const double dx = xpos - self->last_x_;
   const double dy = ypos - self->last_y_;
   self->last_x_ = xpos;
   self->last_y_ = ypos;

   if (self->dragging_) {
       const float sens =0.2f;

       self->yaw_deg_ += sens * static_cast<float>(dx);
       self->pitch_deg_ += sens * static_cast<float>(dy);

       //clamp
       if (self->pitch_deg_ >89.0) self->pitch_deg_ = 89.0f;
       if (self->pitch_deg_ < -89.0) self->pitch_deg_ = -89.0f;
       return;
   }
   if (self->panning_) {
       float dist = self->base_distance_ * self->zoom_;
       float pan_scale = 0.001f * dist;

       float yaw = glm::radians(self->yaw_deg_);

       glm::vec3 cam_right(
           std::cos(yaw),
           -std::sin(yaw),
           0.0f);

       glm::vec3 cam_up_world(0.0f, 0.0f, 1.0f);

       self->view_center_ -=
           cam_right * static_cast<float>(dx) * pan_scale;

       self->view_center_ +=
           cam_up_world * static_cast<float>(dy) * pan_scale;
       return;
   }

}
void OpenGLPointCloudNode::handlePick(double mouse_x, double mouse_y) {
   glm::vec3 ray_o_glm;
   glm::vec3 ray_dir_glm;

   if(!screenRay(mouse_x, mouse_y, ray_o_glm, ray_dir_glm)) {
       std::cout << "[Pick] failed to build screen ray\n";
       return;
   }

    Vec3d ray_o ( ray_o_glm.x,
         ray_o_glm.y,
         ray_o_glm.z);

    Vec3d ray_dir( ray_dir_glm.x, ray_dir_glm.y, ray_dir_glm.z);

    PickPoint picked;

    bool ok =  point_picker_.pickNearestRay(ray_o, ray_dir, 1.0, picked);

    if (!ok) {
        std::cout << "[Pick] no point near ray\n";
        return;
    }

    std::cout
      << "\n===== PICK RESULT =====\n"
      << "layer_id  = " << picked.layer_id_ << "\n"
      << "frame_id  = " << picked.frame_id_ << "\n"
      << "submap_id = " << picked.submap_id_ << "\n"
      << "point_idx = " << picked.p_idx_in_frame_ << "\n"
      << "world     = " << picked.world_p_.transpose() << "\n"
      << "=======================\n";
}

bool OpenGLPointCloudNode::
screenRay(double mouse_x, double mouse_y, glm::vec3 &ray_o, glm::vec3 &ray_dir) const {
   int width;
   int height;

   glfwGetWindowSize(
      window_,
      &width,
      &height
   );

   if (width <= 0 || height <= 0) return false;

   //normal device coordinate
   float ndc_x =static_cast<float>(2.0 * mouse_x /width - 1.0);
   float ndc_y =static_cast<float>(1.0 -  2.0* mouse_y /height);
   glm::mat4 inv_mvp  = glm::inverse(cur_mvp_);

   glm::vec4 near_clip(ndc_x, ndc_y, -1.0f, 1.0f);
   glm::vec4 far_clip(ndc_x,ndc_y, 1.0f, 1.0f);

   //cvt it to world
   glm::vec4  near_world = inv_mvp * near_clip;
   glm::vec4 far_world = inv_mvp * far_clip;

    if (std::abs(near_world.w) < 1e-6f ||
         std::abs(far_world.w) < 1e-6f) {
        return false;
    }

    near_world /= near_world.w;
    far_world /= far_world.w;

    ray_o = glm::vec3(
            near_world.x,
            near_world.y,
            near_world.z);

    glm::vec3 far_p(
        far_world.x,
        far_world.y,
        far_world.z);

    ray_dir =
    glm::normalize(far_p - ray_o);
   return true;
}

void OpenGLPointCloudNode::clearVisualization() {
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_frames_.clear();
    }

    driving_pose_ready_ = false;
    current_frame_points_.clear();
    current_frame_ready_ = false;
    current_frame_dirty_ = false;
    current_frame_count_ = 0;
    for (auto& layer : display_voxels_)
        for (auto& role : layer) role.clear();
    map_points_1_.clear();
    map_points_2_.clear();

    route_points_1_.clear();
    route_points_2_.clear();

    point_picker_.clear();

    map_min_x_ = map_min_y_ = map_min_z_ =
        std::numeric_limits<float>::max();

    map_max_x_ = map_max_y_ = map_max_z_ =
        std::numeric_limits<float>::lowest();

    vbo_ready_1_ = false;
    vbo_ready_2_ = false;

    route_vbo_ready_1_ = false;
    route_vbo_ready_2_ = false;

    num_points_gpu_1_ = 0;
    num_points_gpu_2_ = 0;

    num_route_points_1_ = 0;
    num_route_points_2_ = 0;

    gpu_dirty_1_ = true;
    gpu_dirty_2_ = true;

    route_gpu_dirty_1_ = true;
    route_gpu_dirty_2_ = true;

    auto_fit_pending_ = true;
}

void OpenGLPointCloudNode::onLoopCheckResult(const std_msgs::msg::UInt64::SharedPtr msg) {
    if (!standalone_cloud_path_.empty()) return;
    if (msg->data < active_refresh_id_) return;
    if (msg->data != active_refresh_id_) {
        clearVisualization();
        active_refresh_id_ = msg->data;
        loop_inspection_mode_ = false;
        RCLCPP_INFO(get_logger(), "Loading loop snapshot; Ctrl+L resumes when loading finishes.");
    }
    viewer_ready_pub_->publish(*msg);
}

void OpenGLPointCloudNode::publishLoopResume()
{
    std_msgs::msg::UInt64 resume;
    resume.data = active_refresh_id_;
    loop_resume_pub_->publish(resume);

    loop_inspection_mode_ = false;

    RCLCPP_INFO(
        this->get_logger(),
        "Resume loop closure optimization.");
}

void OpenGLPointCloudNode::key_callback(
    GLFWwindow* window,
    int key,
    int scancode,
    int action,
    int mods)
{
    (void)scancode;
    (void)mods;

    auto* self =
        static_cast<OpenGLPointCloudNode*>(
            glfwGetWindowUserPointer(window));

    if (!self) {
        return;
    }
    if (action == GLFW_PRESS && key == GLFW_KEY_R) self->toggleRecording();
    if (action == GLFW_PRESS && key == GLFW_KEY_V) {
        self->camera_mode_ = self->camera_mode_ == "orbit" ? "driving" : "orbit";
        RCLCPP_INFO(self->get_logger(), "Camera: %s", self->camera_mode_.c_str());
    }
    if (action == GLFW_PRESS &&
      key == GLFW_KEY_L &&
      (mods & GLFW_MOD_CONTROL)){
        if (self->loop_inspection_mode_) {
            self->publishLoopResume();
        }
    }
}

int main(int argc , char **argv) {
 rclcpp::init(argc, argv);
 //create vis node
 auto node = std::make_shared<OpenGLPointCloudNode>();

 //check window
 if(!node->init_window()){
  RCLCPP_FATAL(node->get_logger(), "Failed to initialize OpenGL context");
  rclcpp::shutdown();
  return 1;
}

 rclcpp::WallRate rate(60.0);
 auto start = node->now();

  while (rclcpp::ok() && !glfwWindowShouldClose(node->window())) {
        auto now = node->now();
        float t = static_cast<float>((now - start).seconds());
        node->render_frame(t);
        rclcpp::spin_some(node);
        rate.sleep();
   }

  rclcpp::shutdown();
  return 0;
}