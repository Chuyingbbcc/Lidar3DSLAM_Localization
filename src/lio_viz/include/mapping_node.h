//
// Created by chuchu on 12/12/25.
//

#ifndef MAPPING_NODE_H
#define MAPPING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lio_msgs/msg/frame_data.hpp>
#include <std_msgs/msg/empty.hpp>
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
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr loop_check_pub_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr loop_resume_sub_;
rclcpp::TimerBase::SharedPtr timer_;
std::string pcd_path_;
std::unique_ptr<Frontend> frontend_ptr_;
std::thread slam_thread_;
std::thread optimization_thread_;

//loop pause/resume
std::mutex loop_pause_mutex_;
std::condition_variable loop_pause_cv_;
bool loop_resume_requested_{false};

//replay
std::queue<std::shared_ptr<KeyFrame>>replay_kf_q_;


//helper
void writeVisualizationConfig(const std::string& config_path, MapMode mode);
void onLoopResume(const std_msgs::msg::Empty::SharedPtr msg);
void pauseForLoopInspection();
void runOptimizationMode(const std::string& init_path);
void publishAllKeyFramesForVis(
    const std::map<size_t, std::shared_ptr<KeyFrame>>& kfs);

};



#endif //MAPPING_NODE_H
