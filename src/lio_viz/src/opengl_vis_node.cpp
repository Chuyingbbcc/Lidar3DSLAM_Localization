// opengl_vis_node.cpp
//
// Created by chuchu on 12/12/25.
//

#include "opengl_vis_node.h"
#include "../include/opengl_vis_node.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>

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

    //Todo: hardcode now, replace later with yaml setting file
    layer1_.draw_map_ = true;
    layer1_.draw_route_ = true;
    layer1_.route_pose_type_ = PoseType::LIDAR_NEU;
    layer1_.map_pose_type_ = PoseType::LIDAR_NEU;

    layer2_.draw_map_ = false;
    layer2_.draw_route_ = false;
    layer2_.route_pose_type_ = PoseType::RTK;
    layer2_.map_pose_type_ = PoseType::LIDAR_NEU;
}

OpenGLPointCloudNode::~OpenGLPointCloudNode() {
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

  uniform mat4 u_mvp;
  out float v_intensity;
  void main(){
    gl_Position = u_mvp * vec4(a_position, 1.0);
    gl_PointSize = 1;
    v_intensity =a_intensity;
 }
)";

const char* fs_src =R"(
 #version 330 core

 in float v_intensity;
 uniform vec3 u_color
 out vec4 FragColor;
 void main(){
   FragColor = vec4(u_color, 1.0);
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

void OpenGLPointCloudNode::draw_points_layer(const glm::mat4 &mvp, GLuint vao, bool vbo_ready, size_t num_points, const glm::vec3& color) {
    if (!vbo_ready || num_points ==0) {
        return;
    }
    glUseProgram (shader_program_);

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
   PendingFrame pf;
   pf.frame_id_ = msg->frame_id;
   pf.cloud_path_ = msg->cloud_path;
   pf.lidar_pose_ = poseMsgToGlm(msg->lidar_pose);
   pf.rtk_pose_ = poseMsgToGlm(msg->rtk_pose);
   pf.lidar_pose_neu_ = poseMsgToGlm(msg->lidar_pose_neu);

   pf.has_lidar_pose_ = msg->has_lidar_pose;
   pf.has_rtk_pose_ = msg->has_rtk_pose;
   pf.has_lidar_pose_neu_ = msg->has_lidar_pose_neu;

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
    std::vector<PointVertex>local_points_raw;
    std::vector<PointVertex> local_points;
    if (layer1_.draw_map_ || layer2_.draw_map_) {
       if (!load_point_cloud_local(pf.cloud_path_, local_points_raw)) {
           continue;
       }
       //Todo:: change the voxel size later
       local_points = voxelDownsampleLocal(local_points, 0.2);
    }
    append_frame_to_layer(pf, local_points, layer1_, map_points_1_, route_points_1_);
    append_frame_to_layer(pf, local_points, layer2_, map_points_2_, route_points_2_);
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
  }
}

void OpenGLPointCloudNode::append_frame_to_layer(const PendingFrame &pf, const std::vector<PointVertex> &local_points,
    const RenderLayerConfig &config, std::vector<PointVertex> &map_points, std::vector<glm::vec3> &route_points) {
    if (config.draw_map_) {
       //convert local points to wc
       glm::mat4 T_map;
       if(selectPose(pf, config.map_pose_type_, T_map)) {
           for (const auto& p : local_points) {
               glm::vec4 q = T_map * glm::vec4(p.p_[0], p.p_[1], p.p_[2], 1.0f);
               PointVertex wp =p;
               wp.p_[0] = q.x;
               wp.p_[1] = q.y;
               wp.p_[2] = q.z;

               map_points.push_back(wp);
               update_map_bounds(wp);
           }
       }
    }
    if (config.draw_route_) {
        glm::mat4 T_route;
        if (selectPose(pf, config.route_pose_type_, T_route)) {
            glm::vec3 route_p = getTranslation(T_route);
            if (route_points.empty()|| glm::length(route_points.back()-route_p)> 0.5f) {
              route_points.push_back(route_p);
            }
        }
    }
}

bool OpenGLPointCloudNode::load_point_cloud_local(const std::string &path, std::vector<PointVertex> &out_points) {
    out_points.clear();

    std::ifstream fin(path, std::ios::binary);
    if (!fin.is_open()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot open binary file: %s",
                     path.c_str());
        return false;
    }
    fin.seekg(0, std::ios::end);
    std::streampos nbytes = fin.tellg();
    if (nbytes <= 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "Empty or invalid file: %s",
                     path.c_str());
        return false;
    }
    fin.seekg(0, std::ios::beg);

    const std::size_t total_bytes =
       static_cast<std::size_t>(nbytes);

    const std::size_t num_floats =
        total_bytes / sizeof(float);

    const std::size_t fields_per_point = 3;

    if (num_floats % fields_per_point != 0) {
        RCLCPP_WARN(this->get_logger(),
                    "File size is not perfectly divisible by xyz float layout: %s",
                    path.c_str());
    }

    const size_t num_points = num_floats / fields_per_point;
    if (num_points == 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "No points in file: %s",
                     path.c_str());
        return false;
    }

    std::vector<float>buffer(num_floats);
    fin.read(reinterpret_cast<char*>(buffer.data()), total_bytes);
    if (!fin.good()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to read binary data from %s",
                     path.c_str());
        return false;
    }

    out_points.reserve(num_points);

    for (std::size_t i = 0; i < num_points; ++i) {
        PointVertex p;

        p.p_[0] = buffer[i * fields_per_point + 0];
        p.p_[1] = buffer[i * fields_per_point + 1];
        p.p_[2] = buffer[i * fields_per_point + 2];

        // For now use z as intensity.
        // Later, if your file becomes x y z intensity, change fields_per_point to 4.
        p.intensity_ = p.p_[2];

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

void OpenGLPointCloudNode::merge_points(const std::vector<PointVertex> &new_points) {
   //Todo: add voxel down sampel and also remove overlap
   for (auto& p:  new_points) {
       map_points_.push_back(p);
       update_map_bounds(p);
   }
}

void OpenGLPointCloudNode::update_map_bounds(const PointVertex &p) {
    map_min_x_ = std::min(map_min_x_, p.p_[0]);
    map_min_y_ = std::min(map_min_y_, p.p_[1]);
    map_min_z_ = std::min(map_min_z_, p.p_[2]);

    map_max_x_ = std::max(map_max_x_, p.p_[0]);
    map_max_y_ = std::max(map_max_y_, p.p_[1]);
    map_max_z_ = std::max(map_max_z_, p.p_[2]);
}


void OpenGLPointCloudNode::render_frame(float t){
  if(!window_){
    return;
  }

  consume_pending_frames();
  update_scene_state();

  // events
  glfwPollEvents();

  int w, h;
  glfwGetFramebufferSize(window_, &w, &h);
  float aspect  = (float) w / float(h? h:1);

  glViewport(0, 0, w, h);
  glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::mat4 mvp = compute_mvp(aspect);
    draw_points_layer(
       mvp,
       vao_1_,
       vbo_ready_1_,
       num_points_gpu_1_,
       glm::vec3(1.0f, 1.0f, 1.0f)
       );

    draw_points_layer(
        mvp,
        vao_2_,
        vbo_ready_2_,
        num_points_gpu_2_,
        glm::vec3(0.0f, 1.0f, 0.0f)
        );

    draw_route_layer(
        mvp,
        route_vao_1_,
        route_vbo_ready_1_,
        num_route_points_1_,
        glm::vec3(1,0,0));

    draw_route_layer(
        mvp,
        route_vao_2_,
        route_vbo_ready_2_,
        num_route_points_2_,
        glm::vec3(0,1,0));
  glfwSwapBuffers(window_);
}

void OpenGLPointCloudNode::update_scene_state() {
    if (auto_fit_pending_ && (!map_points_1_.empty() || !map_points_2_.empty())) {
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
     glm::mat4 proj = glm::perspective(
         glm::radians(60.0f),                 // FOV
         aspect,                              // width / height
         0.1f,                                // near plane
         10.0f * base_distance_ + 100.0f       // far plane
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
auto* self = static_cast<OpenGLPointCloudNode*>(glfwGetWindowUserPointer(window));
if(!self) return;
if(button == GLFW_MOUSE_BUTTON_LEFT) {
   if(action == GLFW_PRESS) {
      self->dragging_ =true;
      glfwGetCursorPos(window, &self->last_x_, &self->last_y_);
   }
   else if(action == GLFW_RELEASE) {
      self->dragging_ =false;
   }
}
}

void OpenGLPointCloudNode::cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
  auto* self = static_cast<OpenGLPointCloudNode*>(glfwGetWindowUserPointer(window));
  if (!self)return;
  if(!self->dragging_)return;

   const double dx = xpos - self->last_x_;
   const double dy = ypos - self->last_y_;
   self->last_x_ = xpos;
   self->last_y_ = ypos;

   const float sens =0.2f;

   self->yaw_deg_ += sens * static_cast<float>(dx);
   self->pitch_deg_ += sens * static_cast<float>(dy);

   //clamp
   if (self->pitch_deg_ >89.0) self->pitch_deg_ = 89.0f;
   if (self->pitch_deg_ < -89.0) self->pitch_deg_ = -89.0f;
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