//
// Created by chuchu on 12/12/25.
//

#ifndef MAPPING_NODE_H
#define MAPPING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MappingNode : public rclcpp::Node {
public:
MappingNode();
private:
void on_timer();

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
rclcpp::TimerBase::SharedPtr timer_;
std::string pcd_path_;
};
#endif //MAPPING_NODE_H
