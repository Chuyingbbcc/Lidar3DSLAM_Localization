//
// Created by chuchu on 1/9/26.
//

#pragma once
#include <memory>

#include "Lio/thread_pool.h"
#include "Lio/LidarPostProcessing.h"

#include <mutex>
#include <functional>
#include <thread>

#include "LidarDoneProcessedQueue.h"
#include "lidar_odometry.h"
#include "Lio/imu_buffer.h"
#include "thread_pool.h"
#include  <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ESKF.h"
#include "Imu_data.h"


struct LioOptions {
  bool down_sample_ = true;
  int num_threads_ = 3;
  double down_sample_ratio_ = 0.5;
  double slack_sec_= 1e-2;
  std:: string out_cloud_dir_;
  std:: string out_kf_info_path_;
  ESKF::ESKFOptions eskf_options_;
  LidarOdometry::LoOption lo_options_;
};

class PointCloud;
class KeyFrame;
class LioPipe {
public:
    using PointCloudPre = std::function<void(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr, PointCloud& pc,  const LioOptions& options)>;
    using KeyFrameUpdateCb = std::function<void(const size_t idx , std::shared_ptr<KeyFrame>&)>;
    explicit LioPipe(std::size_t num_thread, double slack_sec, const LioOptions& options);
    void onLidarMsg(const sensor_msgs::msg::PointCloud2& msg, double ts );
    void pushImu(ImuData& data);
    void postProcess();
    void onPreprocess(PointCloudPre callback);
    void onUpdateKeyFrames(KeyFrameUpdateCb callback);
    // void stop();
    void notifyEndOfBag();
    void waitUntilDrained();
    void stopDrain();
    void stopNow();
    void start();
private:
    double watermark() const;
    ThreadPool thread_pool_;
    DoneQueue<PointCloud> dq_;
    LidarReorderBuffer reorder_;
    LioOptions options_;


    mutable std::mutex m_wm_;
    int64_t watermark_t1 = 0;
    PointCloudPre pc_pre_;
    KeyFrameUpdateCb  kf_update_cb_;
    std::thread consumer_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> bag_done_{false};
    std::atomic<int> lidar_inflight_{0};
    bool first_scan_set_ = false;

    SE3d TIL_;
    ESKF eskf_;
    std::vector<ESKFState>eskf_states_;
    IncNDTOdometry  lo_;
    ImuBuffer imu_buffer_;
    int num_frames_ =0;

    void  Predict(std::vector<ImuData>& imu_vect);
    void  Distort();
    void  Align(std::shared_ptr<PointCloud>point_cloud_ptr);
};

