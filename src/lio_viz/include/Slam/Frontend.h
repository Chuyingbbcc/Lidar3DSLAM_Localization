//
// Created by chuchu on 3/18/26.
//
#pragma once
#include <memory>
#include <string>
#include <map>
#include <unordered_map>
#include <mutex>
#include <queue>

#include "io/ros_io_offline.h"
#include "Lio/imu_buffer.h"
#include "Lio/LioPipe.h"
#include "lidar_odometry.h"
#include "Align.h"
#include "ESKF.h"
#include "GPS.h"
#include "KeyFrame.h"



struct FrontendOptions{
   std:: string imu;
   std:: string ros_bag_path_;
   RosIoOffline::IoOptions io_options_;
   LioOptions lio_options_;
};

class Frontend {
    public:
    Frontend();
    Frontend(const std::string& init_path );
    bool init();
    bool Run();
    void stop();

    //Publish
    void publishCloudPath(const std::string& new_path);
    void requestStopAndSave();
    bool hasKfCloudPath() const;
    std::string takeNextPath();

    void publishKeyframe(const std::shared_ptr<KeyFrame>kf_ptr);
    std::shared_ptr<KeyFrame> takeNextKeyFrame();

    std::string getOutKfPath();

    private:
    bool loadFrontendConfig();

    void saveCurrentKeyFrames();
    std::string init_path_ = "";
    bool publish_viewer_ = true;
    bool output_kf_ =true;
    FrontendOptions fe_options_;
    GPS gps_;
    std::map<double, GPSData>gps_data_map_;
    std::map<size_t, std::shared_ptr<KeyFrame>>key_frame_map_;
    std::unique_ptr<RosIoOffline> io_ptr_;
    std::unique_ptr<ImuBuffer> imu_buffer_ptr_;
    std::unique_ptr<LioPipe> lio_pipe_ptr_;

    std::atomic<bool> stop_requested_{false};
    mutable std::mutex kf_mutex_;
    std::queue<std::string>path_queue_;
    bool has_new_kf_path_ = false;


    KeyFrameQueue kf_q_;
    bool has_new_kf = false;

    size_t lidar_count_ = 0;
    size_t max_lidar_count_ = 2000;
};



