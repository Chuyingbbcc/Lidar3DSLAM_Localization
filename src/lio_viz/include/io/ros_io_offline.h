#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class RosIoOffline {
public:
    struct IoOptions {
        std::string bag_folder = "";
        std::vector<std::string> topics;
        bool print_topics_ =true;
    };
    using Pc2Cb = std::function<void(const sensor_msgs::msg::PointCloud2&, int64_t bag_time_ns)>;
    using ImuCb = std::function<void(const sensor_msgs::msg::Imu&, int64_t bag_time_ns )>;
    using TfCb = std::function<void(const tf2_msgs::msg::TFMessage&, int64_t bag_time_ns)>;

    explicit RosIoOffline(IoOptions io_options);
    ~RosIoOffline() = default;

    void onLidar(Pc2Cb cb){pc2_cb_ = std::move(cb);}
    void onImu(ImuCb cb){imu_cb_ = std::move(cb);}
    void onTf(TfCb cb){tf_cb_ = std::move(cb);}

    bool go();

private:
    IoOptions io_options_;
    Pc2Cb pc2_cb_;
    ImuCb imu_cb_;
    TfCb tf_cb_;
};