//
// Created by chuchu on 1/9/26.
//

//#define DEBUG_LIO


#include "Lio/LioPipe.h"
#include "point_cloud_utils.h"
#include <cstddef>
#include <mutex>
#include <memory>
#include <functional>
#include <iostream>
#include <iomanip>
#include <ostream>
#include <thread>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Lio/thread_pool.h>
#include "Lio/imu_buffer.h"
#include "Imu_data.h"
#include "lidar_odometry.h"
#include "common.h"
#include "KeyFrame.h"
#include "../../include/DataType.h"


LioPipe::LioPipe(std::size_t num_thread, double slack, const LioOptions& options): thread_pool_(num_thread), reorder_(slack), options_(options),eskf_(options.eskf_options_), lo_(options.lo_options_) {

}

void LioPipe::start() {
if(running_.exchange(true))return;
consumer_thread_= std::thread([this]{this->postProcess();});
}

void LioPipe::onLidarMsg(const sensor_msgs::msg::PointCloud2& msg,double ts) {
    if(!running_.load(std::memory_order_acquire))return;
    reorder_.onRawSeen(ts);
    //std::cout<<"lidar time: "<< ts << std::endl;
    const auto ops = options_;
    auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);

    lidar_inflight_.fetch_add(1, std::memory_order_acq_rel);
    bool enqueued = thread_pool_.try_enqueue([this, ts, msg_ptr, ops]() {
       PointCloud point_cloud;
       point_cloud.setTime(ts);
       pc_pre_(msg_ptr, point_cloud, ops);
       reorder_.pushProcessed(point_cloud);
       const int left= lidar_inflight_.fetch_sub(1, std::memory_order_acq_rel) -1;
       if(left ==0 && bag_done_.load(std::memory_order_acquire)) {
         reorder_.closeInput();
       }
    });
    if (!enqueued) {
       const int left = lidar_inflight_.fetch_sub(1, std::memory_order_acq_rel) -1;
        if(left ==0 && bag_done_.load(std::memory_order_acquire)) {
            reorder_.closeInput();
        }
    }
}


void LioPipe::pushImu(ImuData& data) {
    imu_buffer_.push(data);
}


void LioPipe::postProcess() {
    double last_t1 = -1.0;
    std::shared_ptr<PointCloud> scan;
    // SO3f last_Pose = SO3f();
    while (reorder_.waitPopOrdered(scan)) {

        std::cout<<"pop out" << num_frames_ << " th lidar"<< std::endl;
        if (!running_.load()) break;
        double t1 =  scan->get_time();
        if (t1 < last_t1) {
           std::cout<< "lidar order messed!" <<std::endl;
           continue;
        }
        if (!imu_buffer_.wait_until(t1, 0.5)) {
            std::cerr << "IMU not ready for t1=" << t1
              << " newest=" << imu_buffer_.latest_time() << "\n";
            continue;
        }
        std::vector<ImuData> imu_vec ={};
        imu_buffer_.extract(imu_vec, t1- 0.1, t1);
        // for(auto& imu : imu_vec) {
        //     if(imu.t <t1-0.1 || imu.t>t1){
        //         std::cerr<< "Imu time invalid! " <<imu.t << "\n";
        //     }
        // }
        //std::cout<<"find " << imu_vec.size() << " imu for lidar "<< t1<< "\n";
        Predict(imu_vec);
        SE3d cur_pose = eskf_.GetCurrentPose();
        //SE3d cur_pose = SE3d();
        size_t kf_idx = -1;
        if(num_frames_ <= 20) {
            kf_idx = lo_.AddCloud(scan,cur_pose, true);
        }
        else {
           kf_idx = lo_.AddCloud(scan, cur_pose, false);
        }
        if(kf_idx == -1 ){
           std::cerr << "Registration failed"<<std::endl;
        }
        if(kf_idx >0) {
            double t = scan->get_time();
            std::shared_ptr<KeyFrame> kf_ptr = std::make_shared<KeyFrame>(t, kf_idx, cur_pose, scan);
            kf_ptr->cloud_path_ = options_.out_cloud_dir_ +"/" + std::to_string(kf_idx) + ".bin";
            kf_ptr-> cloud_size_ = scan->size();
            // write point cloud and unload point cloud,only need write original cloud
            // The point cloud
            SE3d pose_inv = cur_pose.inverse();
            transformCloud(scan, pose_inv);

            if (!writePointCloudToFile(kf_ptr->cloud_path_, scan)) {
                std::cerr << "Failed to write point cloud: "<< kf_ptr->cloud_path_ <<"\n";
                continue;
            }
            kf_update_cb_(kf_idx, kf_ptr);
        }
        scan->clear();
        num_frames_++;
        eskf_.ObserveSE3(cur_pose);
        last_t1 = t1;

#ifdef DEBUG_LIO


        // imu.extract_time_span()
        //eskf.predict()
        //SO3f pose;
        //lo.addCloud(pose)
        //eskf.observe(pose)
        //descrew();
        //last
        // ordered guarantee (should never trigger if slack is sufficient)
        if (scan.get_time() < last_t1) {
            std::cerr << "ORDER VIOLATION: " << scan.get_time() << " < " << last_t1 << "\n";
        }
        last_t1 = scan.get_time();
        // This is where later you'll do:
        // imu_buf.waitUntil(scan.t1), propagate, deskew, ndt align, etc.
        std::cout << "[ORDERED] t1=" << std::fixed << std::setprecision(6)<<scan.get_time() << "\n";
        std::cout <<  " points=" << scan.size()<< "\n";
#endif
    }
     reorder_.closeInput();
     std::cout<< "processed " << num_frames_ << " lidars"<< std::endl;
     std::cout << "postProcess() exit\n";

}

double LioPipe::watermark() const {
    std::lock_guard<std::mutex> lk(m_wm_);
    return watermark_t1;
}


// void LioPipe::stop() {
//     bool was_running_ = running_.exchange(false);
//     if (!was_running_) {
//         return;
//     }
//     reorder_.stop();
//     // dq_.stop();
//    thread_pool_.stop();
//    if (consumer_thread_.joinable()) consumer_thread_.join();
// }

void LioPipe::notifyEndOfBag() {
    bag_done_.store(true, std::memory_order_release);
    int left = lidar_inflight_.load(std::memory_order_acquire);
    if (lidar_inflight_.load(std::memory_order_acquire) ==0) {
       reorder_.closeInput();
    }
}

void LioPipe::waitIfDoneQueueToolLarge() {
   //balance the io and done queue
   reorder_.waitIfReadyTooLarge();
}

void LioPipe::waitUntilDrained() {
   reorder_.waitUntilDone();
   //std::cout<< "recoder successfully closed!" << std::endl;
    if (consumer_thread_.joinable()) consumer_thread_.join();
}

void LioPipe::stopDrain() {
   //notifyEndOfBag();
   waitUntilDrained();
   stopNow();
}

void LioPipe::stopNow() {
   const bool was_running_ = running_.exchange(false, std::memory_order_acq_rel);
   if(!was_running_) {
      return;
   }
   reorder_.stop();
   thread_pool_.stop();
    if(consumer_thread_.joinable()) consumer_thread_.join();
    std::cout<<"consumer join!"<<std::endl;
}

void LioPipe::onPreprocess(PointCloudPre callback) {
  pc_pre_ = std::move(callback);
}

void LioPipe::onUpdateKeyFrames(KeyFrameUpdateCb callback) {
    kf_update_cb_ = std::move(callback);
}

void LioPipe::Predict(std::vector<ImuData>& imu_vect) {
  eskf_states_.clear();
  eskf_states_.emplace_back(eskf_.GetNorminalState());
  for (auto& imu : imu_vect) {
     eskf_.Predict(imu);
     eskf_states_.emplace_back(eskf_.GetNorminalState());
  }
}

void LioPipe::Distort() {
}

void LioPipe::Align(std::shared_ptr<PointCloud>point_cloud_ptr) {
//imu->lidar transform
transformCloud(point_cloud_ptr, TIL_);
//set first scan
if(!first_scan_set_) {
 SE3d pose;
 lo_.AddCloud(point_cloud_ptr, pose);
 first_scan_set_ =true;
}
//ndt.add_cloud
ESKFState cur_state=  eskf_.GetNorminalState();
SE3d cur_pose = SE3d(cur_state.R_, cur_state.p_);
lo_.AddCloud(point_cloud_ptr, cur_pose);
const SE3d lo_pose = cur_pose;
eskf_.ObserveSE3(cur_pose);
return;
}
