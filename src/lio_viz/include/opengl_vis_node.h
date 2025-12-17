#ifndef OPENGL_VIS_NODE_H
#define OPENGL_VIS_NODE_H

#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


#include <vector>
#include <string>
#include <mutex>
using Point3f =Point<float,3>;

struct PointVertex {
    Point3f p_ {};
    float intensity_ =0.0f;
};

class OpenGLPointCloudNode : public rclcpp::Node {
public:
    OpenGLPointCloudNode();
    ~OpenGLPointCloudNode() override;

    bool init_window();
    void render_frame(float t);
    GLFWwindow* window() const { return window_; }

private:
    void on_pcd_path(const std_msgs::msg::String::SharedPtr msg);
    bool load_point_cloud(const std::string &path, std::vector<PointVertex> &out_points);
    bool init_gl_resources();
    void upload_points_to_gpu(const std::vector<PointVertex> &out_points);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::vector<PointVertex> points_;
    bool have_points_{false};
    std::mutex mutex_;

    GLFWwindow* window_{nullptr};


   GLuint vao_;
   GLuint vbo_;
   GLuint shader_program_;
   bool vbo_ready_;
   std::size_t num_points_gpu_;

   // event call back
   float zoom_;
   //drag state
   bool dragging_ = false;
   double last_x_ =0.0;
   double last_y_= 0.0;
   //rotation state
   float yaw_deg_ = 0.0f;
   float pitch_deg_  = 0.0f;

   static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
   static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
   static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
};

#endif // OPENGL_VIS_NODE_H