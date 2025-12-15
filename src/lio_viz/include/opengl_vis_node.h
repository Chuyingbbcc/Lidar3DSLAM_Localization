#ifndef OPENGL_VIS_NODE_H
#define OPENGL_VIS_NODE_H

#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <GLFW/glfw3.h>

#include <vector>
#include <string>
#include <mutex>
using Point3f =Point<float,3>;
class OpenGLPointCloudNode : public rclcpp::Node {
public:
    OpenGLPointCloudNode();
    ~OpenGLPointCloudNode() = default;

    bool init_window();
    void render_frame(float t);
    GLFWwindow* window() const { return window_; }

private:
    void on_pcd_path(const std_msgs::msg::String::SharedPtr msg);
    bool load_point_cloud(const std::string &path, std::vector<Point3f> &out_points);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::vector<Point3f> points_;
    bool have_points_{false};
    std::mutex mutex_;

    GLFWwindow* window_{nullptr};

   // event call back
   float zoom_;

   static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

};

#endif // OPENGL_VIS_NODE_H