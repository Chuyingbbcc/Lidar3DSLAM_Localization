//
// Created by chuchu on 12/12/25.
//
#include "mapping_node.h"
#include <iostream>

using namespace std::chrono_literals;

MappingNode::MappingNode(): Node("mapping_node") {
pub_ = create_publisher<std_msgs::msg::String>("pcd_path", 10);

pcd_path_ = this->declare_parameter<std::string>(
      "pcd_path",
      "/media/chuchu/Extreme Pro/kitti/data_odometry_velodyne/dataset/sequences/01/velodyne/000000.bin"   // default, override via --ros-args
);
timer_ =this->create_wall_timer(
1s,
std::bind(&MappingNode::on_timer, this)
);
 RCLCPP_INFO(this->get_logger(), "Mapping node publishing path: %s", pcd_path_.c_str());

}

void MappingNode::on_timer(){
  auto msg = std_msgs::msg::String();
  msg.data = pcd_path_;
  pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    3000,
    "Publishing pcd_path: %s",
    pcd_path_.c_str()
  );
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}