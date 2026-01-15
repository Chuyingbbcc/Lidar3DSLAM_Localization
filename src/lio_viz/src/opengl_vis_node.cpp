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
      have_points_(false),
      zoom_(5.0f)
{
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "pcd_path",
        10,
        std::bind(&OpenGLPointCloudNode::on_pcd_path, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Visualization node started. Waiting for pcd_path messages...");
}

OpenGLPointCloudNode::~OpenGLPointCloudNode() {
  if(vbo_){
  glDeleteBuffers(1, &vbo_);
  }
  if(vao_){
   glDeleteVertexArrays(1, &vao_);
  }
  if(shader_program_){
    glDeleteShader(shader_program_);
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
 out vec4 FragColor;

 vec3 colormap(float t){
  t= clamp(t, 0.0, 1.0);
  if (t < 0.33) {
    // dark blue -> cyan
    float u = t / 0.33;
    return mix(vec3(0.0, 0.0, 0.3), vec3(0.0, 1.0, 1.0), u);
  } else if (t < 0.66) {
    // cyan -> yellow
    float u = (t - 0.33) / 0.33;
    return mix(vec3(0.0, 1.0, 1.0), vec3(1.0, 1.0, 0.0), u);
  } else {
    // yellow -> white
    float u = (t - 0.66) / 0.34;
    return mix(vec3(1.0, 1.0, 0.0), vec3(1.0, 1.0, 1.0), u);
  }
 }
 void main(){
   float t = v_intensity;
   vec3 color =colormap(t);
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
if(!shader_program_){
return false;
}

//setup vbo & vao
glGenVertexArrays(1, &vao_);
glGenBuffers(1, &vbo_);
if (!vao_ || !vbo_) {
        RCLCPP_ERROR(logger, "Failed to create VAO/VBO");
        return false;
}
glBindVertexArray(vao_);
glBindBuffer(GL_ARRAY_BUFFER, vbo_);

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

//unbind
    /*👍 We unbind because OpenGL is a global state machine:

    Leaving VAO/VBO bound makes them vulnerable to accidental modification.

    Unbinding ensures no later code changes attribute state unintentionally.*/

glBindBuffer(GL_ARRAY_BUFFER, 0);
glBindVertexArray(0);

vbo_ready_ = false;
num_points_gpu_ =0;

return true;
}

void OpenGLPointCloudNode::upload_points_to_gpu(const std::vector<PointVertex> &pts){
  if(!vbo_)return;

  //bind ptr data
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, pts.size() * sizeof(PointVertex), pts.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  num_points_gpu_ = pts.size();
  vbo_ready_ = (num_points_gpu_ > 0);

  RCLCPP_INFO(this->get_logger(), "Uploaded %zu points to GPU", num_points_gpu_);
}

void OpenGLPointCloudNode::on_pcd_path(const std_msgs::msg::String::SharedPtr msg) {
    std::vector<PointVertex> new_points;

    if (!load_point_cloud(msg->data, new_points)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load file: %s", msg->data.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        points_ = std::move(new_points);
        have_points_ = true;
    }

    // Upload to GPU
    upload_points_to_gpu(points_);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu points", points_.size());
}

bool OpenGLPointCloudNode::load_point_cloud(const std::string &path, std::vector<PointVertex> &out)
{
    // Open as binary file
    std::ifstream fin(path, std::ios::binary);
    if (!fin.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open binary file: %s", path.c_str());
        return false;
    }

    // Get file size in bytes
    fin.seekg(0, std::ios::end);
    std::streampos nbytes = fin.tellg();
    if (nbytes <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Empty or invalid file: %s", path.c_str());
        return false;
    }
    fin.seekg(0, std::ios::beg);

    std::size_t total_bytes  = static_cast<std::size_t>(nbytes);
    std::size_t num_floats   = total_bytes / sizeof(float);

    // Typical Velodyne/KITTI: x,y,z,intensity -> 4 floats per point
    // But we also allow 3 floats/point (xyz only) as a fallback.
    std::size_t fields_per_point = 0;
    if (num_floats % 5 == 0) {
        fields_per_point = 5;
    } else if (num_floats % 4 == 0) {
        fields_per_point = 4;
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Unexpected binary layout in %s (floats=%zu, not divisible by 3 or 4)",
                     path.c_str(), num_floats);
        return false;
    }

    std::size_t num_points = num_floats / fields_per_point;
    if (num_points == 0) {
        RCLCPP_ERROR(this->get_logger(), "No points in file: %s", path.c_str());
        return false;
    }

    std::vector<float> buffer(num_floats);
    fin.read(reinterpret_cast<char*>(buffer.data()), total_bytes);
    if (!fin.good()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read binary data from %s", path.c_str());
        return false;
    }

    // Convert to raw xyz points (ignore intensity / extra fields)
    std::vector<PointVertex> raw;
    raw.reserve(num_points);

    for (std::size_t i = 0; i < num_points; ++i) {
        const float x = buffer[i * fields_per_point + 0];
        const float y = buffer[i * fields_per_point + 1];
        const float z = buffer[i * fields_per_point + 2];
        float intensity =1.0f;
        if (fields_per_point >=4) {
          intensity = buffer[i*fields_per_point+3];
        }
        else {
          intensity = z;
        }
        raw.push_back({{x, y, z},intensity});
    }


    float minx = std::numeric_limits<float>::max();
    float miny = std::numeric_limits<float>::max();
    float minz = std::numeric_limits<float>::max();
    float maxx = std::numeric_limits<float>::lowest();
    float maxy = std::numeric_limits<float>::lowest();
    float maxz = std::numeric_limits<float>::lowest();

    for (auto &p : raw) {
        minx = std::min(minx, p.p_[0]); maxx = std::max(maxx, p.p_[0]);
        miny = std::min(miny, p.p_[1]); maxy = std::max(maxy, p.p_[1]);
        minz = std::min(minz, p.p_[2]); maxz = std::max(maxz, p.p_[2]);
    }

    float cx = 0.5f * (minx + maxx);
    float cy = 0.5f * (miny + maxy);
    float cz = 0.5f * (minz + maxz);

    float range_x = maxx - minx;
    float range_y = maxy - miny;
    float range_z = maxz - minz;
    float max_range = std::max(range_x, std::max(range_y, range_z));
    if (max_range <= 0.0f) max_range = 1.0f;

    float scale = 2.0f / max_range;

    out.clear();
    out.reserve(raw.size());

    for (auto &p : raw) {
        Point3f point = p.p_;
        PointVertex vertex;
        point[0] = (point[0] - cx) * scale;
        point[1] = (point[1] - cy) * scale;
        point[2] = (point[2] - cz) * scale;
        vertex.p_ = point;
        if(fields_per_point >=4) {
            vertex.intensity_ = p.intensity_;
        }
        else {
           vertex.intensity_ = point[2];
        }
        out.push_back(vertex);
    }
    return true;
}

void OpenGLPointCloudNode::render_frame(float t){
  if(!window_){
    return;
  }
  // events
  glfwPollEvents();

  int w, h;
  glfwGetFramebufferSize(window_, &w, &h);
  float aspect  = (float) w / float(h? h:1);

  glViewport(0, 0, w, h);
  glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if(!vbo_ready_ ||num_points_gpu_ ==0){
   glfwSwapBuffers(window_);
   return;
  }
  //Set up mvp
  glm::mat4 proj = glm::perspective(
  glm::radians(60.0f),
  aspect,
  0.1f,
  100.0f);
 glm::vec3 cam_pos(0.0f, zoom_ , 0.0); // z is point out  of the screen
 glm::vec3 cam_target(0.0f, 0.0f, 0.0f);
 glm::vec3 cam_up(0.0f, 0.0f, 1.0f); // y is on up
 glm::mat4 view = glm::lookAt(cam_pos, cam_target, cam_up);

 //glm::mat4 model = glm::rotate(glm::mat4(1.0f), glm::radians(t*10.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  glm ::mat4 model(1.0f);
    model = glm::rotate(model, glm::radians(yaw_deg_), glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::rotate(model, glm::radians(pitch_deg_), glm::vec3(1.0f, 0.0f, 0.0f));
 glm::mat4 mvp = proj * view * model;

 //draw
 glUseProgram(shader_program_);
 GLint loc_mvp = glGetUniformLocation(shader_program_, "u_mvp");
 glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, glm::value_ptr(mvp));

  glBindVertexArray(vao_);
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(num_points_gpu_));
  glBindVertexArray(0);

  glUseProgram(0);
  glfwSwapBuffers(window_);
}

void OpenGLPointCloudNode::scroll_callback(GLFWwindow* window,
                                           double xoffset,
                                           double yoffset){
    (void)xoffset;  // unused

    // Recover the C++ object from the GLFW window
    auto* self = static_cast<OpenGLPointCloudNode*>(
        glfwGetWindowUserPointer(window));
    if (!self) return;

    // yoffset > 0 : scroll up, yoffset < 0 : scroll down
    self->zoom_ -= static_cast<float>(yoffset) * 0.5f;  // adjust speed here

    // Clamp zoom between [1, 50]
    if (self->zoom_ < 1.0f)  self->zoom_ = 1.0f;
    if (self->zoom_ > 50.0f) self->zoom_ = 50.0f;
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