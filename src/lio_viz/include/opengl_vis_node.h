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
class OpenGLPointCloudNode : public rclcpp::Node {
public:
    OpenGLPointCloudNode();
    ~OpenGLPointCloudNode() override;

    bool init_window();
    void render_frame(float t);
    GLFWwindow* window() const { return window_; }

private:
    void on_pcd_path(const std_msgs::msg::String::SharedPtr msg);
    bool load_point_cloud(const std::string &path, std::vector<Point3f> &out_points);
    bool init_gl_resources();
    void upload_points_to_gpu(const std::vector<Point3f> &out_points);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::vector<Point3f> points_;
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

   static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

};

#endif // OPENGL_VIS_NODE_H