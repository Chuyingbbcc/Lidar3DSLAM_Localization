//
// Created by chuchu on 1/9/26.
//

#pragma once
#include <memory>

#include "Lio/thread_pool.h"
#include "Lio/LidarPreProcessing.h"

#include <mutex>
#include <functional>

#include "LidarDoneProcessedQueue.h"
#include "thread_pool.h"
#include  <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "../point_cloud_utils.h"

struct LioOptions {
  float voxel_size = 0.5;
  bool voxel_down_sample = true;
};

class PointCloud;
class LidarPreProcessing {
public:
    using PointCloudPre = std::function<void(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr, PointCloud& pc,  const LioOptions& options)>;
    explicit LidarPreProcessing(std::size_t num_thread);
    void onLidarMsg(const sensor_msgs::msg::PointCloud2& msg, int64_t ts,  const LioOptions& options);
    void postProcess();
    void onPreprocess(PointCloudPre callback);
    void stop();
private:
    double watermark() const;
    ThreadPool thread_pool_;
    DoneQueue<PointCloud> dq_;
    mutable std::mutex m_wm_;
    int64_t watermark_t1 = 0;
    PointCloudPre pc_pre_;
};



