#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>


class RosIoOffline {
public:
    struct IoOptions {
        std::string bag_folder = "";
        std::vector<std::string> topics;
        bool print_topics_ =true;
        bool down_sample_  = true;
    };
    RosIoOffline(){};
    RosIoOffline(const std::string& path ): bag_path_(path) {}
    IoOptions io_options_;
    using Pc2Cb = std::function<void(const sensor_msgs::msg::PointCloud2&, double bag_time_ns)>;
    using ImuCb = std::function<void(const sensor_msgs::msg::Imu&, double bag_time_ns )>;
    using TfCb = std::function<void(const tf2_msgs::msg::TFMessage&, double bag_time_ns)>;
    using GpsCb =std::function<void(const sensor_msgs::msg::NavSatFix&, double bag_time_ns)>;
    explicit RosIoOffline(IoOptions io_options);
    ~RosIoOffline() = default;

    void onLidar(Pc2Cb cb){pc2_cb_ = std::move(cb);}
    void onImu(ImuCb cb){imu_cb_ = std::move(cb);}
    void onTf(TfCb cb){tf_cb_ = std::move(cb);}
    void onGps(GpsCb cb){gps_cb_ = std::move(cb);}

    bool go();

private:
    std::string bag_path_;
    Pc2Cb pc2_cb_;
    ImuCb imu_cb_;
    TfCb tf_cb_;
    GpsCb gps_cb_;
    double last_lidar_time_{0.0};
    double last_imu_time_{0.0};
    double last_tf_time_{0.0};
    double last_gps_time_{0.0};

    double extractTime(const std_msgs::msg::Header& header, int64_t bag_time_ns, double& last_t);
};