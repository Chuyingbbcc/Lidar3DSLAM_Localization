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
std::cout << "Odometry mode: " << (options_.use_imu_ ? "LIO (IMU enabled)" : "LO (LiDAR only)") << std::endl;
if(running_.exchange(true))return;
consumer_thread_= std::thread([this]{
    try { postProcess(); } catch (...) { recordWorkerFailure(); }
});
}

void LioPipe::onLidarMsg(const sensor_msgs::msg::PointCloud2& msg,double ts) {
    if(!running_.load(std::memory_order_acquire))return;
    reorder_.onRawSeen(ts);
    //std::cout<<"lidar time: "<< ts << std::endl;
    const auto ops = options_;
    auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);

    lidar_inflight_.fetch_add(1, std::memory_order_acq_rel);
    bool enqueued = thread_pool_.try_enqueue([this, ts, msg_ptr, ops]() {
       try {
       PointCloud point_cloud;
       point_cloud.setTime(ts);
       pc_pre_(msg_ptr, point_cloud, ops);
       reorder_.pushProcessed(point_cloud);
       } catch (...) { recordWorkerFailure(); }
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
    if (options_.use_imu_) imu_buffer_.push(data);
}


void LioPipe::postProcess() {
    double last_t1 = -1.0;
    std::shared_ptr<PointCloud> scan;
    // SO3f last_Pose = SO3f();
    while (reorder_.waitPopOrdered(scan)) {
        //std::cout<<"pop out" << num_frames_ << " th lidar"<< std::endl;
        if (!running_.load()) break;
        double t1 =  scan->get_time();
        if (t1 < last_t1) {
           std::cout<< "lidar order messed!" <<std::endl;
           continue;
        }
        SE3d cur_pose;
        bool enough_prediction = false;
        if (options_.use_imu_) {
            if (!scan->hasPointTiming()) {
                throw std::runtime_error("IMU processing requires explicit per-point scan timing");
            }
            if (!imu_buffer_.wait_until(t1, 0.5)) {
                throw std::runtime_error("IMU coverage does not reach scan end; stopping LIO");
            }
            const double state_time = eskf_.GetNorminalState().t_;
            const double scan_start = static_cast<double>(scan->scanStartNs()) * 1e-9;
            const double integrate_from = state_time > 0 ? state_time : scan_start;
            // Validate the whole acquisition interval as well as the state propagation interval.
            std::vector<ImuData> imu_vec;
            if (!imu_buffer_.extractCovered(imu_vec, std::min(scan_start, integrate_from), t1,
                                            5 * options_.eskf_options_.imu_dt_)) {
                throw std::runtime_error("Missing IMU coverage or excessive gap across scan interval; stopping LIO");
            }
            if (state_time <= 0) {
                // Seed only the clock; gravity/bias initialization is a separate step.
                eskf_.Predict(imu_vec.front());
            }
            Predict(imu_vec);
            enough_prediction = true;
            cur_pose = eskf_.GetCurrentPose();
            imu_buffer_.dropOlderThan(scan_start - 0.1);
        }
        // LO uses the odometry motion guess without waiting for IMU data.
        const bool use_guess = !options_.use_imu_ || num_frames_ <= 20 || !enough_prediction;
        size_t kf_idx = lo_.AddCloud(scan, cur_pose, use_guess);
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
        if (options_.use_imu_) eskf_.ObserveSE3(cur_pose);
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
   rethrowWorkerFailure();
   reorder_.waitIfReadyTooLarge();
   rethrowWorkerFailure();
}

void LioPipe::waitUntilDrained() {
   reorder_.waitUntilDone();
   //std::cout<< "recoder successfully closed!" << std::endl;
    if (consumer_thread_.joinable()) consumer_thread_.join();
    rethrowWorkerFailure();
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
     if (imu.t <= eskf_.GetNorminalState().t_) continue;
     if (!eskf_.Predict(imu)) throw std::runtime_error("IMU prediction rejected a covered interval");
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

void LioPipe::recordWorkerFailure() {
    {
        std::lock_guard<std::mutex> lock(failure_mutex_);
        if (!worker_failure_) worker_failure_ = std::current_exception();
    }
    reorder_.stop();
}

void LioPipe::rethrowWorkerFailure() {
    std::exception_ptr failure;
    {
        std::lock_guard<std::mutex> lock(failure_mutex_);
        failure = worker_failure_;
    }
    if (failure) std::rethrow_exception(failure);
}
