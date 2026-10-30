//
// Created by chuchu on 1/9/26.
//

#pragma once
#include <memory>
#include <exception>

#include "Lio/thread_pool.h"
#include "Lio/LidarPostProcessing.h"

#include <mutex>
#include <functional>
#include <thread>
#include <vector>
#include <cstdint>

#include "LidarDoneProcessedQueue.h"
#include "lidar_odometry.h"
#include "Lio/imu_buffer.h"
#include "thread_pool.h"
#include  <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ESKF.h"
#include "Imu_data.h"
#include "Lio/LidarImuFrames.h"
#include "Lio/FixedMapLocalization.h"


struct ImuPredictionRecord;

struct LioOptions {
  FixedMapOptions localization_;
  bool use_imu_ = true;
  bool synchronous_preprocessing_ = false;
  bool exclude_lidar_x_band_ = false;
  double excluded_lidar_x_min_ = 0.0;
  double excluded_lidar_x_max_ = 0.0;
  bool deskew_enabled_ = true; // Independent of IMU prediction and filter updates.
  bool imu_recovery_enabled_ = true;
  size_t imu_recovery_good_scans_ = 3;
  bool imu_prediction_log_ = true;
  std::string imu_prediction_log_path_;  // Empty: out_cloud_dir_/imu_prediction.csv.
  SE3d T_imu_lidar_;  // p_imu = T_imu_lidar_ * p_lidar.
  double imu_acc_scale_ = 1.0;  // Input acceleration to m/s^2.
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
    void waitIfDoneQueueToolLarge();
    void stopDrain();
    void stopNow();
    void start();
    void rethrowWorkerFailure();
private:
    void recordWorkerFailure();
    std::mutex failure_mutex_;
    std::exception_ptr worker_failure_;
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
    std::mutex performance_mutex_;
    std::vector<uint64_t> queue_wait_us_;
    std::vector<uint64_t> preprocess_us_;
    std::vector<uint64_t> callback_to_preprocess_us_;

    LidarImuFrames frames_;
    ESKF eskf_;
    bool imu_initialized_ = false;
    double imu_initialization_end_ = 0;
    std::vector<ESKFState>eskf_states_;
    IncNDTOdometry  lo_;
    std::unique_ptr<FixedMapLocalization> localizer_;
    ImuBuffer imu_buffer_;
    int num_frames_ =0;

    void Predict(double scan_start, double scan_end, ImuPredictionRecord* diagnostic = nullptr);
};

