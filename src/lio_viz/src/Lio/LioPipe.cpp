//
// Created by chuchu on 1/9/26.
//

//#define DEBUG_LIO


#include "Lio/LioPipe.h"
#include "Lio/Deskew.h"
#include "Lio/ImuRecovery.h"
#include "Lio/GravityAlignment.h"
#include "Lio/ImuPredictionLog.h"
#include "point_cloud_utils.h"
#include <cstddef>
#include <mutex>
#include <memory>
#include <functional>
#include <iostream>
#include <iomanip>
#include <ostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Lio/thread_pool.h>
#include "Lio/imu_buffer.h"
#include "Imu_data.h"
#include "lidar_odometry.h"
#include "common.h"
#include "KeyFrame.h"
#include "../../include/DataType.h"


LioPipe::LioPipe(std::size_t num_thread, double slack, const LioOptions& options): thread_pool_(num_thread), reorder_(slack), options_(options),frames_(options.T_imu_lidar_),eskf_(options.eskf_options_), lo_(options.lo_options_) {
    if(options_.localization_.enabled) {
        localizer_=std::make_unique<FixedMapLocalization>(options_.localization_,options_.lo_options_.inc_opt_);
        std::cout<<"[Localization test] fixed reference points="<<localizer_->referencePoints()<<"\n";
    }
}

void LioPipe::start() {
std::cout << "Odometry mode: " << (options_.use_imu_ ? "LIO (IMU enabled)" : "LO (LiDAR only)") << std::endl;
if (options_.use_imu_) std::cout << "Point deskew: " << (options_.deskew_enabled_ ? "enabled" : "disabled (IMU prediction/filter still enabled)") << "\n";
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

    if(options_.localization_.enabled || options_.synchronous_preprocessing_) {
        // Offline replay can request ordered preprocessing to prevent worker
        // queue saturation and completion jitter from dropping/reordering scans.
        PointCloud cloud;cloud.setTime(ts);pc_pre_(msg_ptr,cloud,ops);
        reorder_.pushProcessed(cloud);
        return;
    }

    lidar_inflight_.fetch_add(1, std::memory_order_acq_rel);
    const auto callback_time=std::chrono::steady_clock::now();
    bool enqueued = thread_pool_.try_enqueue([this, ts, msg_ptr, ops, callback_time]() {
       try {
       const auto worker_time=std::chrono::steady_clock::now();
       PointCloud point_cloud;
       point_cloud.setTime(ts);
       pc_pre_(msg_ptr, point_cloud, ops);
       const auto preprocess_end=std::chrono::steady_clock::now();
       const auto queue_us=std::chrono::duration_cast<std::chrono::microseconds>(worker_time-callback_time).count();
       const auto preprocess_us=std::chrono::duration_cast<std::chrono::microseconds>(preprocess_end-worker_time).count();
       const auto total_us=std::chrono::duration_cast<std::chrono::microseconds>(preprocess_end-callback_time).count();
       { std::lock_guard<std::mutex> lock(performance_mutex_);
         queue_wait_us_.push_back(static_cast<uint64_t>(queue_us));
         preprocess_us_.push_back(static_cast<uint64_t>(preprocess_us));
         callback_to_preprocess_us_.push_back(static_cast<uint64_t>(total_us)); }
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
    ImuPredictionLog prediction_log;
    bool prediction_log_open = false;
    auto log_record = [&](const ImuPredictionRecord& record) {
        if (!options_.use_imu_ || !options_.imu_prediction_log_) return;
        if (!prediction_log_open) {
            const auto automatic = options_.out_cloud_dir_.empty() ? std::string("imu_prediction.csv") : options_.out_cloud_dir_ + "/imu_prediction.csv";
            const auto path = options_.imu_prediction_log_path_.empty() ? automatic : options_.imu_prediction_log_path_;
            prediction_log.open(path);
            const auto vertical_path = options_.out_cloud_dir_.empty() ? std::string("vertical_diagnostics.csv") : options_.out_cloud_dir_ + "/vertical_diagnostics.csv";
            prediction_log.openVertical(vertical_path);
            std::cout << "[Vertical diagnostics] " << vertical_path << "\n";
            prediction_log_open = true;
            std::cout << "[IMU diagnostics] " << path << "\n";
        }
        auto annotated = record;
        annotated.detail = std::string("deskew_requested=") + (options_.deskew_enabled_ ? "true" : "false") +
            "; deskew_applied=" + (record.deskew_applied ? "true" : "false") + "; " + record.detail;
        prediction_log.write(annotated);
    };
    bool gravity_alignment_saved = false;
    auto save_gravity_alignment = [&](const ImuPredictionRecord& record) {
        if (gravity_alignment_saved) return;
        const Vec3d up = record.map_from_world * (-record.prior_gravity.normalized());
        const auto alignment = GravityAlignment::fromUp(up, record.scan_end_ns);
        const std::string path = options_.out_cloud_dir_.empty() ? "gravity_alignment.yaml" : options_.out_cloud_dir_ + "/gravity_alignment.yaml";
        alignment.save(path);
        gravity_alignment_saved = true;
        std::cout << "[Gravity alignment] fixed display tilt=" << alignment.tiltDegrees() << " deg; " << path << "\n";
    };
    ImuRecovery recovery(options_.imu_recovery_good_scans_);
    SE3d previous_registered_imu;
    double previous_registered_time = -1;
    size_t scan_index = 0;
    double last_t1 = -1.0;
    std::shared_ptr<PointCloud> scan;
    // SO3f last_Pose = SO3f();
    while (reorder_.waitPopOrdered(scan)) {
        //std::cout<<"pop out" << num_frames_ << " th lidar"<< std::endl;
        if (!running_.load()) break;
        ImuPredictionRecord diagnostic;
        diagnostic.scan_index = ++scan_index;
        if(localizer_ && scan_index<options_.localization_.start_frame)continue;
        if(localizer_ && options_.localization_.end_frame && scan_index>options_.localization_.end_frame)continue;
        diagnostic.scan_start_ns = scan->scanStartNs();
        diagnostic.scan_end_ns = scan->scanEndNs();
        diagnostic.points_before_deskew = scan->size();
        if (!scan->empty() && options_.imu_prediction_log_) {
            Vec3d low = Vec3d::Constant(std::numeric_limits<double>::infinity());
            Vec3d high = -low;
            for (const auto& point : *scan) {
                const Vec3d p(point.x,point.y,point.z);
                low = low.cwiseMin(p); high = high.cwiseMax(p);
            }
            diagnostic.raw_cloud_span = high-low;
        }
        try {
        double t1 =  scan->get_time();
        if (t1 < last_t1) {
           if(localizer_)throw std::runtime_error("Localization scan ordering violation");
           diagnostic.event = "scan_skipped";
           diagnostic.detail = "Older scan arrived after a newer processed scan";
           log_record(diagnostic);
           std::cout<< "lidar order messed!" <<std::endl;
           continue;
        }
        SE3d cur_pose;
        bool enough_prediction = false;
        bool imu_interval_covered = false;
        if (options_.use_imu_) {
            if (!scan->hasPointTiming()) {
                throw std::runtime_error("IMU processing requires explicit per-point scan timing");
            }
            if (!imu_initialized_) {
                std::vector<ImuData> initial;
                if (!imu_buffer_.initialWindow(initial, options_.eskf_options_.imu_initialization.duration_sec)) {
                    if (bag_done_.load()) throw std::runtime_error("Bag ended before stationary IMU initialization completed");
                    continue; // Warm-up scans are not mapped; the reader can keep filling the IMU buffer.
                }
                const auto result = eskf_.InitializeStationary(initial);
                if (!result.valid) throw std::runtime_error("IMU initialization failed: " + result.reason);
                auto initial_record = diagnostic;
                initial_record.event = "initialization";
                initial_record.has_update = true;
                initial_record.updated = eskf_.GetNorminalState();
                initial_record.updated_gravity = eskf_.GetGravity();
                initial_record.detail = "stationary_samples=" + std::to_string(result.samples);
                log_record(initial_record);
                imu_initialized_ = true;
                imu_initialization_end_ = eskf_.GetNorminalState().t_;
                std::cout << "[IMU init] samples=" << result.samples << " duration=" << result.duration_sec
                          << " gyro bias=" << result.gyro_bias.transpose()
                          << " gravity(initial map frame)=" << result.gravity.transpose()
                          << " acc std=" << result.acc_std << " gyro std=" << result.gyro_std << "\n";
            }
            const double scan_start = static_cast<double>(scan->scanStartNs()) * 1e-9;
            if (scan_start < imu_initialization_end_) continue;
            diagnostic.has_prior = true;
            diagnostic.prior = eskf_.GetNorminalState();
            diagnostic.prior_gravity = eskf_.GetGravity();
            if(localizer_ && !frames_.anchored()) {
                frames_.anchor(eskf_.GetCurrentPose(),options_.localization_.initial_pose);
                // The supplied initial pose belongs to this first selected scan,
                // not to the earlier stationary calibration window.
                eskf_.ReanchorAfterGap(t1,frames_.imuPose(options_.localization_.initial_pose),Vec3d::Zero());
                diagnostic.prior=eskf_.GetNorminalState();
            }
            const bool reached_end = imu_buffer_.wait_until(t1, 0.5);
            std::vector<ImuData> coverage_check;
            const bool covered = reached_end && imu_buffer_.extractCovered(coverage_check,
                std::min(scan_start, eskf_.GetNorminalState().t_), t1, 5 * options_.eskf_options_.imu_dt_);
            imu_interval_covered=covered;
            if (!covered && !options_.imu_recovery_enabled_)
                throw std::runtime_error("Missing IMU propagation coverage or excessive gap; stopping LIO");
            const bool was_fallback = recovery.fallback();
            if (recovery.useImu(covered)) {
                diagnostic.propagation_start_s = eskf_.GetNorminalState().t_;
                Predict(scan_start, t1, &diagnostic);
                const SE3d predicted_imu_pose = eskf_.GetCurrentPose();
                if (!frames_.anchored()) frames_.anchor(predicted_imu_pose);
                cur_pose = frames_.lidarPose(predicted_imu_pose);
                diagnostic.has_map_rotation = true;
                diagnostic.map_from_world = cur_pose.so3() * frames_.imuFromLidar().so3().inverse() * predicted_imu_pose.so3().inverse();
                diagnostic.prior_lidar = frames_.lidarPose(SE3d(diagnostic.prior.R_,diagnostic.prior.p_));
                save_gravity_alignment(diagnostic);
                diagnostic.has_prediction = true;
                diagnostic.prediction = eskf_.GetNorminalState();
                diagnostic.predicted_covariance_diagonal = eskf_.GetCovarianceDiagonal();
                diagnostic.has_covariance = true;
                diagnostic.predicted_gravity = eskf_.GetGravity();
                diagnostic.predicted_lidar = cur_pose;
                diagnostic.points_registered = scan->size();
                diagnostic.trajectory_states = eskf_states_.size();
                diagnostic.trajectory_max_dt_s = 0;
                for (size_t i = 1; i < eskf_states_.size(); ++i)
                    diagnostic.trajectory_max_dt_s = std::max(diagnostic.trajectory_max_dt_s, eskf_states_[i].t_ - eskf_states_[i-1].t_);
                if (options_.deskew_enabled_) {
                    deskewToScanEnd(*scan, eskf_states_, frames_.imuFromLidar(), 5 * options_.eskf_options_.imu_dt_);
                    diagnostic.deskew_applied = scan->isDeskewed();
                }
                if (options_.down_sample_) downsampleTimedCloud(*scan, options_.down_sample_ratio_);
                if (scan->empty()) throw std::runtime_error("No points remain after point preprocessing and sampling");
                enough_prediction = true;
                diagnostic.points_registered = scan->size();
                if (was_fallback) {
                    auto recovered = diagnostic;
                    recovered.event = "imu_recovered";
                    recovered.detail = "Complete IMU coverage restored; resumed after LO state reanchoring";
                    log_record(recovered);
                    std::cout << "[IMU recovery] resumed inertial prediction; deskew=" << (options_.deskew_enabled_ ? "enabled" : "disabled") << "\n";
                }
            } else {
                if (!was_fallback) std::cout << "[IMU recovery] missing coverage; continuing with LO without deskew\n";
                if (!frames_.anchored()) frames_.anchor(eskf_.GetCurrentPose());
                diagnostic.has_map_rotation = true;
                diagnostic.prior_lidar = frames_.lidarPose(SE3d(diagnostic.prior.R_,diagnostic.prior.p_));
                diagnostic.map_from_world = diagnostic.prior_lidar.so3() * frames_.imuFromLidar().so3().inverse() * diagnostic.prior.R_.inverse();
                save_gravity_alignment(diagnostic);
                if (options_.down_sample_) downsampleTimedCloud(*scan, options_.down_sample_ratio_);
                if (scan->empty()) throw std::runtime_error("No points remain in LO recovery scan");
                diagnostic.points_registered = scan->size();
                diagnostic.detail = covered ? "Healthy IMU interval; waiting for recovery confirmation" :
                    "Missing or gapped IMU interval; raw LO without deskew";
                diagnostic.detail += "; healthy_scans=" + std::to_string(recovery.goodScans()) + "/" + std::to_string(options_.imu_recovery_good_scans_);
            }
            imu_buffer_.dropOlderThan(scan_start - 0.1);
        }
        // LO uses the odometry motion guess without waiting for IMU data.
        const bool use_guess = !options_.use_imu_ || (!localizer_ && num_frames_ <= 20) || !enough_prediction;
        if (options_.use_imu_ && enough_prediction) {
            diagnostic.event = "prediction";
            diagnostic.imu_guess_requested = !use_guess;
            log_record(diagnostic); // Persist the uncorrected prediction before registration.
        }
        bool localization_accepted=false;
        size_t kf_idx=0;
        if(localizer_) {
            SE3d prediction=cur_pose;
            const SE3d* prediction_ptr=enough_prediction?&prediction:nullptr;
            if(options_.localization_.seed_source=="lo") {
                auto lo_scan=std::make_shared<PointCloud>(*scan);
                SE3d lo_pose;
                lo_.AddCloud(lo_scan,lo_pose,true);
                // LO is initialized at identity on the first selected scan.
                // Express its independent trajectory in the configured map frame.
                if(num_frames_>0) {
                    prediction=options_.localization_.initial_pose*lo_pose;
                    prediction_ptr=&prediction;
                } else prediction_ptr=nullptr;
            }
            const auto result=localizer_->registerScan(scan,prediction_ptr,scan_index,imu_interval_covered);
            cur_pose=result.registered;
            localization_accepted=result.accepted;
            if(result.accepted)kf_idx=size_t(num_frames_)+1;
            diagnostic.ndt_alignment_ok=result.aligned;
            diagnostic.imu_guess_used=result.seed_type=="imu";
            diagnostic.map_seed=false;
            diagnostic.detail+="; localization="+result.reason+"; seed="+result.seed_type;
        } else kf_idx = lo_.AddCloud(scan, cur_pose, use_guess);
        if (options_.use_imu_) {
            diagnostic.has_registration = true;
            diagnostic.registered_lidar = cur_pose;
            if(!localizer_) {
                diagnostic.imu_guess_used = lo_.lastUsedImuPrediction();
                diagnostic.ndt_alignment_ok = lo_.lastAlignmentSucceeded();
                diagnostic.map_seed = num_frames_ == 0;
            }
        }
        if(kf_idx == -1 ){
           std::cerr << "Registration failed"<<std::endl;
        }
        if(kf_idx >0) {
            double t = scan->get_time();
            std::shared_ptr<KeyFrame> kf_ptr = std::make_shared<KeyFrame>(t, kf_idx, cur_pose, scan);
            if(localizer_) {
                kf_ptr->lidar_pose_neu_=cur_pose;
                kf_ptr->rtk_valid_=false;kf_ptr->rtk_inlier_=false;
            }
            kf_ptr->cloud_path_ = options_.out_cloud_dir_ +"/" + std::to_string(kf_idx) + ".bin";
            kf_ptr-> cloud_size_ = scan->size();
            // write point cloud and unload point cloud,only need write original cloud
            // The point cloud
            SE3d pose_inv = cur_pose.inverse();
            if(!localizer_)transformCloud(scan, pose_inv);

            if (!writePointCloudToFile(kf_ptr->cloud_path_, scan)) {
                std::cerr << "Failed to write point cloud: "<< kf_ptr->cloud_path_ <<"\n";
                diagnostic.event = "cloud_write_failed";
                diagnostic.detail = kf_ptr->cloud_path_;
                log_record(diagnostic);
                if(localizer_)throw std::runtime_error("Cannot persist localization scan output");
                continue;
            }
            kf_update_cb_(kf_idx, kf_ptr);
        }
        scan->clear();
        num_frames_++;
        if (options_.use_imu_) {
            const SE3d registered_imu = frames_.imuPose(cur_pose);
            const bool registration_valid = localizer_?localization_accepted:(lo_.lastAlignmentSucceeded() || diagnostic.map_seed);
            if (enough_prediction && (!localizer_ || registration_valid)) {
                eskf_.ObserveSE3(registered_imu);
                diagnostic.event = "update";
            } else if(enough_prediction) {
                diagnostic.event="localization_rejected"; // Propagation retained; no rejected map pose enters ESKF.
            } else {
                diagnostic.event = "lo_fallback";
                if (registration_valid) {
                    Vec3d velocity = Vec3d::Zero();
                    if (previous_registered_time >= 0 && t1 > previous_registered_time)
                        velocity = (registered_imu.translation() - previous_registered_imu.translation()) / (t1 - previous_registered_time);
                    eskf_.ReanchorAfterGap(t1, registered_imu, velocity);
                    diagnostic.detail += "; filter pose/time/velocity reseeded from LO; bias/gravity retained";
                } else {
                    recovery.registrationFailed();
                    diagnostic.detail += "; LO alignment failed; IMU recovery remains suspended";
                }
            }
            if (registration_valid) {
                previous_registered_imu = registered_imu;
                previous_registered_time = t1;
            }
            diagnostic.has_update = localizer_?registration_valid:(enough_prediction || registration_valid);
            diagnostic.has_updated_lidar = diagnostic.has_update;
            diagnostic.updated = eskf_.GetNorminalState();
            diagnostic.updated_covariance_diagonal = eskf_.GetCovarianceDiagonal();
            diagnostic.has_covariance = true;
            diagnostic.updated_gravity = eskf_.GetGravity();
            diagnostic.updated_lidar = frames_.lidarPose(eskf_.GetCurrentPose());
            log_record(diagnostic);
        }
        last_t1 = t1;
        } catch (const std::exception& error) {
            diagnostic.event = "error";
            diagnostic.detail = error.what();
            log_record(diagnostic);
            throw;
        }

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
    auto report=[](const char* name,std::vector<uint64_t> values) {
        if(values.empty()) { std::cout<<"[PERF] "<<name<<" count=0\n"; return; }
        std::sort(values.begin(),values.end());
        auto pct=[&](double p){return values[static_cast<size_t>(p*(values.size()-1))];};
        std::cout<<"[PERF] "<<name<<" count="<<values.size()<<" mean_us="
                 <<(std::accumulate(values.begin(),values.end(),uint64_t{0})/values.size())
                 <<" p50_us="<<pct(.50)<<" p95_us="<<pct(.95)<<" p99_us="<<pct(.99)
                 <<" max_us="<<values.back()<<"\n";
    };
    std::vector<uint64_t> queue_wait,preprocess,total;
    { std::lock_guard<std::mutex> lock(performance_mutex_); queue_wait=queue_wait_us_; preprocess=preprocess_us_; total=callback_to_preprocess_us_; }
    std::cout<<"[PERF] worker_queue_capacity=64 worker_queue_max="<<thread_pool_.max_queued()
             <<" worker_queue_rejections="<<thread_pool_.rejected()
             <<" reorder_heap_max="<<reorder_.maxHeapDepth()
             <<" ready_queue_max="<<reorder_.maxReadyDepth()<<"\n";
    report("worker_queue_wait",std::move(queue_wait));
    report("lidar_preprocess",std::move(preprocess));
    report("callback_to_preprocess",std::move(total));
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

void LioPipe::Predict(double scan_start, double scan_end, ImuPredictionRecord* diagnostic) {
    const auto initial = eskf_.GetNorminalState();
    const double max_gap = 5 * options_.eskf_options_.imu_dt_;
    std::vector<ImuData> propagation, overlap;
    // Validate before advancing the filter. Both pieces have interpolated endpoints.
    if (scan_end < initial.t_ || (scan_end > initial.t_ &&
        !imu_buffer_.extractCovered(propagation, initial.t_, scan_end, max_gap))) {
        throw std::runtime_error("Missing IMU propagation coverage or excessive gap; stopping LIO");
    }
    if (scan_start < initial.t_) {
        if (!imu_buffer_.extractCovered(overlap, scan_start, initial.t_, max_gap))
            throw std::runtime_error("Missing IMU coverage across overlapping scans; stopping LIO");
        eskf_states_ = reconstructImuOverlap(initial, overlap, eskf_.GetGravity(), max_gap);
    } else {
        eskf_states_.assign(1, initial);
    }
    for (const auto& imu : propagation) {
        if (imu.t <= initial.t_) continue;
        if (diagnostic) {
            const auto before = eskf_.GetNorminalState();
            const double dt = imu.t-before.t_;
            const Vec3d specific_force = before.R_*(imu.acc-before.ba_);
            diagnostic->propagation_duration_s += dt;
            diagnostic->mean_specific_force_w += specific_force*dt;
            diagnostic->mean_acceleration_w += (specific_force+eskf_.GetGravity())*dt;
            diagnostic->max_input_acc_norm = std::max(diagnostic->max_input_acc_norm,imu.acc.norm());
            diagnostic->max_input_gyro_norm = std::max(diagnostic->max_input_gyro_norm,imu.gyro.norm());
            ++diagnostic->propagation_samples;
        }
        if (!eskf_.Predict(imu)) throw std::runtime_error("IMU prediction rejected a covered interval");
        eskf_states_.emplace_back(eskf_.GetNorminalState());
    }
    if (diagnostic && diagnostic->propagation_duration_s > 0) {
        diagnostic->mean_specific_force_w /= diagnostic->propagation_duration_s;
        diagnostic->mean_acceleration_w /= diagnostic->propagation_duration_s;
        diagnostic->has_propagation_stats = true;
    }
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
