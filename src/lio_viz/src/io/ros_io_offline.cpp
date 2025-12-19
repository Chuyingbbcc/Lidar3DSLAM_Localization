//
// Created by chuchu on 12/17/25.
//

#include "io/ros_io_offline.h"
#include "iostream"

#include <unordered_set>

#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

#include <rosbag2_cpp/reader.hpp>

#if __has_include(<rosbag2_cpp/readers/sequential_reader.hpp>)
  #include <rosbag2_cpp/readers/sequential_reader.hpp>
  namespace rb2_readers = rosbag2_cpp::readers;
#elif __has_include(<rosbag2_cpp/sequential_reader.hpp>)
#include <rosbag2_cpp/sequential_reader.hpp>
namespace rb2_readers = rosbag2_cpp;
#else
#error "Cannot find SequentialReader header. Search /opt/ros/$ROS_DISTRO/include/rosbag2_cpp for sequential_reader.hpp"
#endif

#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <rosbag2_storage/serialized_bag_message.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>


RosIoOffline::RosIoOffline(IoOptions io_options): io_options_(io_options) {

}

bool RosIoOffline::go() {
  // Foxy: Reader requires an impl
  auto reader_impl = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  rosbag2_cpp::Reader reader(std::move(reader_impl));

  try {
    rosbag2_cpp::StorageOptions storage_options;
    storage_options.uri = io_options_.bag_folder;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return false;
  }

  if (io_options_.print_topics_) {  // <-- note underscore based on your error
    auto topics = reader.get_all_topics_and_types();
    std::cout << "[RosbagOfflineReader] Topics in bag:\n";
    for (const auto& t : topics) {
      std::cout << "  " << t.name << "  [" << t.type << "]\n";
    }
  }

  // If you have a topics list in io_options_ (rename if needed)
  if (!io_options_.topics.empty()) {
    rosbag2_storage::StorageFilter filter;
    filter.topics = io_options_.topics;
    reader.set_filter(filter);
  }

  // Serializers
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_ser;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;

  // Extra safety filter (optional)
  std::unordered_set<std::string> want;
  for (const auto& t : io_options_.topics) want.insert(t);

  size_t total = 0, pc2_n = 0, imu_n = 0, odom_n = 0, tf_n = 0;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    ++total;

    const std::string& topic_name = bag_msg->topic_name;
    const int64_t bag_time_ns = bag_msg->time_stamp;

    if (!want.empty() && want.find(topic_name) == want.end()) {
      continue;
    }

    if (pc2_cb_ && (topic_name == "kitti/velo" )) {
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      pc2_ser.deserialize_message(&serialized, &msg);
      ++pc2_n;
      pc2_cb_(msg, bag_time_ns);
      continue;
    }

    if (imu_cb_ && (topic_name == "kitti/oxts/imu" )) {
      sensor_msgs::msg::Imu msg;
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      imu_ser.deserialize_message(&serialized, &msg);
      ++imu_n;
      imu_cb_(msg, bag_time_ns);
      continue;
    }

    // // If you don't have odom_cb_, comment this out or add the member.
    // if (odom_cb_ && (topic_name == "/odom" || topic_name == "/odometry")) {
    //   nav_msgs::msg::Odometry msg;
    //   rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    //   odom_ser.deserialize_message(&serialized, &msg);
    //   ++odom_n;
    //   odom_cb_(msg, bag_time_ns);
    //   continue;
    // }

    if (tf_cb_ && (topic_name == "/tf" || topic_name == "/tf_static")) {
      tf2_msgs::msg::TFMessage msg;
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      tf_ser.deserialize_message(&serialized, &msg);
      ++tf_n;
      tf_cb_(msg, bag_time_ns);
      continue;
    }
  }

  std::cout << "[RosbagOfflineReader] Done. total=" << total
            << " pc2=" << pc2_n
            << " imu=" << imu_n
            << " odom=" << odom_n
            << " tf=" << tf_n
            << "\n";
  return true;
}