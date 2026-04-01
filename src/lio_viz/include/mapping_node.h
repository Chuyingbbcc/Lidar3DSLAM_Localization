//
// Created by chuchu on 12/12/25.
//

#ifndef MAPPING_NODE_H
#define MAPPING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "Slam/Frontend.h"

class MappingNode : public rclcpp::Node {
public:
MappingNode();
~MappingNode();
private:
void on_timer();

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
rclcpp::TimerBase::SharedPtr timer_;
std::string pcd_path_;
std::unique_ptr<Frontend> frontend_ptr_;
std::thread slam_thread_;

};


#endif //MAPPING_NODE_H
