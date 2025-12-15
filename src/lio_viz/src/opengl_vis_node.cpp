// opengl_vis_node.cpp
//
// Created by chuchu on 12/12/25.
//

#include "opengl_vis_node.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>

using namespace std;

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

bool OpenGLPointCloudNode::init_window() {
    if (!glfwInit()) {
        RCLCPP_ERROR(this->get_logger(), "GLFW init failed");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    window_ = glfwCreateWindow(2000,2000 , "Point Cloud Viz", nullptr, nullptr);
    if (!window_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create window");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);

    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, &OpenGLPointCloudNode::scroll_callback);

    return true;
}

void OpenGLPointCloudNode::on_pcd_path(const std_msgs::msg::String::SharedPtr msg) {
    std::vector<Point3f> new_points;

    if (!load_point_cloud(msg->data, new_points)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load file: %s", msg->data.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        points_ = std::move(new_points);
        have_points_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points", points_.size());
}

bool OpenGLPointCloudNode::load_point_cloud(const std::string &path, std::vector<Point3f> &out)
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
    if (num_floats % 4 == 0) {
        fields_per_point = 4;
    } else if (num_floats % 3 == 0) {
        fields_per_point = 3;
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
    std::vector<Point3f> raw;
    raw.reserve(num_points);

    for (std::size_t i = 0; i < num_points; ++i) {
        const float x = buffer[i * fields_per_point + 0];
        const float y = buffer[i * fields_per_point + 1];
        const float z = buffer[i * fields_per_point + 2];
        raw.push_back({x, y, z});
    }


    float minx = std::numeric_limits<float>::max();
    float miny = std::numeric_limits<float>::max();
    float minz = std::numeric_limits<float>::max();
    float maxx = std::numeric_limits<float>::lowest();
    float maxy = std::numeric_limits<float>::lowest();
    float maxz = std::numeric_limits<float>::lowest();

    for (auto &p : raw) {
        minx = std::min(minx, p[0]); maxx = std::max(maxx, p[0]);
        miny = std::min(miny, p[1]); maxy = std::max(maxy, p[1]);
        minz = std::min(minz, p[2]); maxz = std::max(maxz, p[2]);
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
        out.push_back({
            (p[0] - cx) * scale,
            (p[1] - cy) * scale,
            (p[2] - cz) * scale
        });
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

  //Projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  float fov = 60.0f * M_PI / 180.0f;
  float f = 1.0f /std::tan(fov/ 2.0f);

  float z_near = 0.1f;
  float z_far  = 100.0f;

  float proj[16] = {
     f / aspect, 0, 0, 0,
     0, f, 0, 0,
     0, 0, (z_far + z_near) / (z_near - z_far), -1,
     0, 0, (2 * z_far * z_near) / (z_near - z_far), 0
  };
  glLoadMatrixf(proj);

  //modelview MATRIX
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0f, 0.0f, -zoom_);
  glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
  glRotatef(t * 10.0f, 0.0f, 1.0f, 0.0f);

  vector<Point3f> ptrs;
  {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!have_points_){
    glfwSwapBuffers(window_);
    return;
  }
   ptrs = points_;
  }
  //rendering
  glBegin(GL_POINTS);
  for(auto& p: ptrs){
      float c = 0.5f + 0.5f * p[2]; // z in [-1,1]
      if (c < 0.0f) c = 0.0f;
      if (c > 1.0f) c = 1.0f;
      glColor3f(c, 1.0f - c, 0.5f);
      glVertex3f(p[0], p[1], p[2]);
  }
  glEnd();
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