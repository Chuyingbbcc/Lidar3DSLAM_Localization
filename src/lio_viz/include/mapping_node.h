//
// Created by chuchu on 12/12/25.
//

#ifndef MAPPING_NODE_H
#define MAPPING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lio_msgs/msg/frame_data.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "Slam/Frontend.h"
#include "Slam/Optimization.h"

enum class  MapMode {
   frontend = 0,
   optimization = 1,
   replay_frontend =2,
   replay_optimization = 3,
};

class MappingNode : public rclcpp::Node {
public:
MappingNode();
~MappingNode();
   void stop_and_save();
private:
void on_timer();
void publish_frame();

std::string mode_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
rclcpp::Publisher<lio_msgs::msg::FrameData>::SharedPtr frame_pub_;
rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr loop_check_pub_;
rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr viewer_ready_sub_;
rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr viewer_loaded_sub_;
rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr loop_resume_sub_;
rclcpp::TimerBase::SharedPtr timer_;
std::string pcd_path_;
std::unique_ptr<Frontend> frontend_ptr_;
std::thread slam_thread_;
std::thread optimization_thread_;

//loop pause/resume
std::mutex loop_pause_mutex_;
std::condition_variable loop_pause_cv_;
bool loop_resume_requested_{false};
bool viewer_ready_{false};
bool viewer_rendered_{false};
bool awaiting_inspection_{false};
bool snapshot_sent_{false};
uint64_t refresh_id_{0};

//replay
std::mutex replay_mutex_;
bool replay_enabled_{true};
uint64_t replay_refresh_id_{0};
std::queue<std::shared_ptr<KeyFrame>>replay_kf_q_;


//helper
void writeVisualizationConfig(const std::string& config_path, MapMode mode);
void onLoopResume(const std_msgs::msg::UInt64::SharedPtr msg);
void pauseForLoopInspection();
void onViewerReady(const std_msgs::msg::UInt64::SharedPtr msg);
void onViewerLoaded(const std_msgs::msg::UInt64::SharedPtr msg);
void refreshForLoopInspection(const std::map<size_t, std::shared_ptr<KeyFrame>>& kfs, size_t count);
void publishKeyFrameForVis(const std::shared_ptr<KeyFrame>& kf,
                          uint64_t refresh_id = 0, bool refresh_last = false);
void runOptimizationMode(const std::string& init_path);
void publishAllKeyFramesForVis(
    const std::map<size_t, std::shared_ptr<KeyFrame>>& kfs);

};



#endif //MAPPING_NODE_H
