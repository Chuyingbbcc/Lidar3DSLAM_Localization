//
// Created by chuchu on 12/12/25.
//
#include "mapping_node.h"
#include <iostream>
#include <thread>

#include "io/ros_io_offline.cpp"

using namespace std::chrono_literals;

MappingNode::MappingNode(): Node("mapping_node") {
pub_ = create_publisher<std_msgs::msg::String>("pcd_path", 10);
std::string init_path  = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/Config/config.yaml";
frontend_ptr_ = std::make_unique<Frontend>(init_path);
//run frontend
slam_thread_ = std::thread([this]() {
try {
  frontend_ptr_->init();
  frontend_ptr_->Run();
}
catch (const std::exception& e) {
  RCLCPP_ERROR(this->get_logger(), "Frontend exception: %s", e.what());
}
catch(...) {
  RCLCPP_ERROR(this->get_logger(), "Unknown frontend exception.");
}
});

timer_ =this->create_wall_timer(
20ms,
std::bind(&MappingNode::on_timer, this)
);
 RCLCPP_INFO(this->get_logger(), "Mapping node publishing path: %s", pcd_path_.c_str());
}

MappingNode::~MappingNode() {
  RCLCPP_INFO(this->get_logger(), "Destructor start");

  if (timer_) {
    RCLCPP_INFO(this->get_logger(), "Cancelling timer");
    timer_->cancel();
  }

  if (frontend_ptr_) {
    RCLCPP_INFO(this->get_logger(), "Stopping frontend");
    frontend_ptr_->stop();
    RCLCPP_INFO(this->get_logger(), "Frontend stopped");
  }

  if (slam_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining slam thread");
    slam_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Slam thread joined");
  }

  RCLCPP_INFO(this->get_logger(), "Destructor done");
}

void MappingNode::on_timer(){
  if(!frontend_ptr_) {
    return;
  }
  if(frontend_ptr_->hasKfCloudPath()) {
    std::string cur_path = frontend_ptr_->takeNextPath();
    std::cout<<"publish cloud path: " << cur_path<<std::endl;
    auto msg = std_msgs::msg::String();
    msg.data = cur_path;
    pub_->publish(msg);
    // RCLCPP_INFO_THROTTLE(
    //   this->get_logger(),
    //   *this->get_clock(),
    //   2000,
    //   "Publishing pcd_path: %s",
    //   cur_path.c_str()
    // );
  }
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  std::cout<<"map node shut down!";
  return 0;
}