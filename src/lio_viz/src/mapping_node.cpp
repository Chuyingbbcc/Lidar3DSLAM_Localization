//
// Created by chuchu on 12/12/25.
//
#include "mapping_node.h"
#include <iostream>
#include "io/ros_io_offline.cpp"

using namespace std::chrono_literals;

MappingNode::MappingNode(): Node("mapping_node") {
pub_ = create_publisher<std_msgs::msg::String>("pcd_path", 10);

pcd_path_ = this->declare_parameter<std::string>(
      "pcd_path",
      "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/cloud.bin"   // default, override via --ros-args
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

  // const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";
  // RosIoOffline::IoOptions options;
  // options.bag_folder = ros_bag_path;
  // RosIoOffline io(options);
  // io.onImu([](const sensor_msgs::msg::Imu& imu, int64_t t_ns) {
  // (void)t_ns;
  // // push into imu buffer...
  //  std::cout << "imu stamp " << imu.header.stamp.sec << "\n";
  // });
  //
  // io.onLidar([](const sensor_msgs::msg::PointCloud2& cloud, int64_t t_ns) {
  //     (void)t_ns;
  //     const size_t npts = (size_t)cloud.width * (size_t)cloud.height;
  //     std::cout << "cloud points=" << npts << " frame=" << cloud.header.frame_id << "\n";
  //     // do mapping step...
  // });
  //
  // if (!io.go()) {
  //       std::cerr << "Failed reading bag.\n";
  // }

  rclcpp::shutdown();
  return 0;
}