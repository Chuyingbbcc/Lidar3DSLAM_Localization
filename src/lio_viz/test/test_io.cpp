//
// Created by chuchu on 12/17/25.
//
#include <gtest/gtest.h>


#include "../include/point_cloud_utils.h"
#include  "io/ros_io_offline.h"
#include  "point_cloud_utils.h"
#include  <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>


class RosTestEnv: public::testing::Test {
protected:
    static void SetUpTestSuite() {
       if(!rclcpp::ok()) {
          int argc =0;
          rclcpp::init(argc, nullptr);
       }
    }
    static void TearDownTestSuite() {
      if (rclcpp::ok()) {
         rclcpp::shutdown();
      }
    }
};

TEST(IO, TESTIO) {
  const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";

  RosIoOffline::IoOptions options;
  options.bag_folder = ros_bag_path;
  RosIoOffline io(options);

  io.onImu([](const sensor_msgs::msg::Imu& imu, int64_t t_ns) {
  (void)t_ns;
  // push into imu buffer...
   std::cout << "imu stamp " << imu.header.stamp.sec << "\n";
  });
  io.onLidar([](const sensor_msgs::msg::PointCloud2& cloud, int64_t t_ns) {
      PointCloud point_cloud;
      pc2ToPointCloudXYZIT(cloud, point_cloud);

     for (size_t i = 0; i < std::min<size_t>(point_cloud.size(), 5); ++i) {
        const auto& p = point_cloud[i];
        std::cout << "  [" << i << "] "
               << "x=" << p.x << " y=" << p.y << " z=" << p.z
               << " i=" << p.intensity << "\n" ;}
  });
  if(!io.go()) {
     std::cerr<< "Failed rendering bag.\n";
  }
  rclcpp::shutdown();
  EXPECT_TRUE(true);
}