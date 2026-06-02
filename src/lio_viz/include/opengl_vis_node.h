#ifndef OPENGL_VIS_NODE_H
#define OPENGL_VIS_NODE_H

#include "common.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lio_msgs/msg/frame_data.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "glad.h"
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include "vis_utils.h"


#include <vector>
#include <string>
#include <mutex>
#include <queue>
#include <yaml-cpp/yaml.h>

using Point3f =Point<float,3>;

struct PointVertex {
    Point3f p_ {};
    float intensity_ =0.0f;
};

enum class  PoseType {
   LIDAR,
   RTK,
   LIDAR_NEU
};

struct PendingFrame {
   uint64_t frame_id_ = 0;
   std::string cloud_path_ = "";
   //glm::vec3 position;

   glm::mat4 lidar_pose_ = glm::mat4(1.0f);
   glm::mat4 rtk_pose_ = glm::mat4(1.0f);
   glm::mat4 lidar_pose_neu_ = glm::mat4(1.0f);

    bool has_lidar_pose_ = true;
    bool has_rtk_pose_ = true;
    bool has_lidar_pose_neu_ = true;
};

struct RenderLayerConfig {
    size_t id_ = 0;
    PoseType map_pose_type_ = PoseType::LIDAR;
    PoseType route_pose_type_ = PoseType::RTK;
    bool draw_map_ = true;
    bool draw_route_ = true;
    glm::vec3 map_color_ = {0.0f, 0.0f, 0.0f};
    glm::vec3 route_color_ = {0.0f, 0.0f, 0.0f};
};

class OpenGLPointCloudNode : public rclcpp::Node {
public:
    OpenGLPointCloudNode();
    ~OpenGLPointCloudNode() override;

    bool init_window();

    //render frame
    void render_frame(float t);
    void update_scene_state();
    glm::mat4 compute_mvp(float aspect) const;
    GLFWwindow* window() const { return window_; }

private:
    void on_pcd_path(const std_msgs::msg::String::SharedPtr msg);
    void on_key_frame_callback(const lio_msgs::msg::FrameData::SharedPtr msg);
    void consume_pending_frames();
    void append_frame_to_layer(const PendingFrame& pf, const std::vector<PointVertex>& local_points, const RenderLayerConfig& config, std::vector<PointVertex>& map_points, std::vector<glm::vec3>& route_points);
    bool load_point_cloud_local(const std::string &path, std::vector<PointVertex>& out_points);
    void merge_points(const std::vector<PointVertex>& new_points);
    void update_scene_bounds(const glm::vec3& p);
    bool init_gl_resources();
    void upload_points_to_gpu(const std::vector<PointVertex>& pts, GLuint vbo, bool& ready, size_t& count);
    void upload_route_to_gpu(const std::vector<glm::vec3>&route_pts, GLuint route_vbo, bool& ready, size_t& count);

    void draw_points_layer(const glm::mat4 &mvp, GLuint vao, bool vbo_ready, size_t num_points, const glm::vec3& color);
    void draw_route_layer(const glm::mat4& mvp, GLuint vao, bool vbo_ready, size_t num_points_route, const glm::vec3& color);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Subscription<lio_msgs::msg::FrameData>::SharedPtr frame_sub_;

    //Todo: might delete later
    // std::vector<PointVertex> points_;
    // bool have_points_{false};

    bool initialized_ = false;
    RenderLayerConfig layer1_;
    RenderLayerConfig layer2_;

    std::vector<PointVertex> map_points_1_;
    std::vector<PointVertex> map_points_2_;

    std::vector<glm::vec3> route_points_1_;
    std::vector<glm::vec3> route_points_2_;

    std::mutex mutex_;

    std::queue<std::string>pending_paths_;
    std::vector<PointVertex>map_points_;


    bool gpu_dirty_1_ = false;
    bool gpu_dirty_2_ = false;

    bool route_gpu_dirty_1_ = false;
    bool route_gpu_dirty_2_ = false;

    //pending frames
    std::mutex pending_mutex_;
    std::deque<PendingFrame> pending_frames_;

    //route
    std::mutex route_mutex_;
    //std::vector<glm::vec3>route_points_;

    //point Picker
    PointPicker point_picker_;


    //bounding and scale
    float map_min_x_ =  std::numeric_limits<float>::max();
    float map_min_y_ =  std::numeric_limits<float>::max();
    float map_min_z_ =  std::numeric_limits<float>::max();

    float map_max_x_ = -std::numeric_limits<float>::max();
    float map_max_y_ = -std::numeric_limits<float>::max();
    float map_max_z_ = -std::numeric_limits<float>::max();

    GLFWwindow* window_{nullptr};
   // GLuint vao_ =0;
   // GLuint vbo_ =0;
   //  //route
   //  GLuint route_vao_ = 0;
   //  GLuint route_vbo_ = 0;
    GLuint vao_1_ =0;
    GLuint vbo_1_ =0;

    GLuint vao_2_ =0;
    GLuint vbo_2_ =0;

    GLuint route_vao_1_ =0;
    GLuint route_vbo_1_ =0;

    GLuint route_vao_2_ =0;
    GLuint route_vbo_2_ =0;


   GLuint shader_program_ =0;
   GLuint route_shader_program_ =0;

   // bool vbo_ready_ =false;
   // std::size_t num_points_gpu_ =0 ;
   bool vbo_ready_1_ = false;
   std::size_t num_points_gpu_1_ = 0;
   bool vbo_ready_2_ = false;
   std::size_t num_points_gpu_2_ = 0;

   // bool route_vbo_ready_ =false;
   // std::size_t num_route_points_ =0;
   bool route_vbo_ready_1_ =false;
   std::size_t num_route_points_1_ =0;

   bool route_vbo_ready_2_ =false;
   std::size_t num_route_points_2_ =0;

   // event call back
   float zoom_;
   //drag state
   bool panning_ = false;
   bool dragging_ = false;
   double last_x_ =0.0;
   double last_y_= 0.0;
   //rotation state
   float yaw_deg_ = 0.0f;
   float pitch_deg_  = 0.0f;

   glm::vec3 view_center_{0.0f,0.0f,0.0f};
   float base_distance_ = 10.0f;

   bool auto_fit_pending_ = false;

   double voxel_size_ = 0.1f;

    glm::mat4 cur_mvp_{1.0f};
    double mouse_down_x_ = 0.0;
    double mouse_down_y_ = 0.0;

    void handlePick(double mouse_x, double mouse_y);
    bool screenRay(double mouse_x, double mouse_y, glm::vec3& ray_o, glm::vec3& ray_dir) const;


   //event call back
   static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
   static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
   static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);

    //helper functions
    void compute_view_params();
    std::vector<PointVertex> voxelDownsampleLocal(const std::vector<PointVertex>&input, double voxel_size);
    bool selectPose(const PendingFrame& pf, PoseType pose, glm::mat4& T)const;
    glm::vec3 getTranslation(const glm::mat4& T)const;
    bool setupPointBuffers(GLuint& vao, GLuint&vbo);
    bool setupRouteBuffers(GLuint& route_vao, GLuint& route_vbo);

    //config helper
    bool loadVisNodeConfig(const std::string& path);
    void loadLayerConfig(const YAML::Node& node, RenderLayerConfig& layer);
    PoseType parsePoseType(const std::string& s);
};



glm::mat4 poseMsgToGlm(const geometry_msgs::msg::Pose& pose) {
    Eigen::Quaterniond q(
          pose.orientation.w,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z
      );
    q.normalize();

    Eigen::Matrix3d R = q.toRotationMatrix();

    glm::mat4 T(1.0f);

    // GLM is column-major: T[col][row]
    T[0][0] = R(0,0);
    T[0][1] = R(1,0);
    T[0][2] = R(2,0);

    T[1][0] = R(0,1);
    T[1][1] = R(1,1);
    T[1][2] = R(2,1);

    T[2][0] = R(0,2);
    T[2][1] = R(1,2);
    T[2][2] = R(2,2);

    T[3][0] = pose.position.x;
    T[3][1] = pose.position.y;
    T[3][2] = pose.position.z;
    T[3][3] = 1.0f;
    return T;
}


#endif // OPENGL_VIS_NODE_H