//
// Created by chuchu on 3/18/26.
//

#include "Slam/Frontend.h"
#include "io/ros_io_offline.h"
#include "Lio/imu_buffer.h"
#include "Lio/LioPipe.h"
#include "lidar_odometry.h"
#include "point_cloud_utils.h"
#include "Align.h"
#include "ESKF.h"
#include "GPS.h"
#include "KeyFrame.h"
#include  <rclcpp/rclcpp.hpp>

#include <iostream>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include "../../include/KeyFrame.h"

using namespace std;
Frontend::Frontend(const std::string &init_path): init_path_(init_path) {
}



bool Frontend::init() {
    if (!loadFrontendConfig()) {
        cout<<"Fail to load frontend config"<<endl;
        return false;
    }
    std::cout<<"ros bag: "<< fe_options_.ros_bag_path_<<endl;
    io_ptr_ = make_unique<RosIoOffline>(fe_options_.ros_bag_path_);
    io_ptr_->io_options_ = fe_options_.io_options_;
    imu_buffer_ptr_ = make_unique<ImuBuffer>();
    lio_pipe_ptr_ = make_unique<LioPipe>(fe_options_.lio_options_.num_threads_, fe_options_.lio_options_.slack_sec_, fe_options_.lio_options_);
   return true;
}

bool Frontend::Run() {
    // size_t l_count =0;
    try {
        lio_pipe_ptr_-> onPreprocess(
          [](std::shared_ptr<sensor_msgs::msg::PointCloud2>msg_ptr, PointCloud& pc, const LioOptions& options) {
          pc2ToPointCloudXYZIT(msg_ptr, pc, options.down_sample_ratio_, options.down_sample_);
        }
       );

        lio_pipe_ptr_-> onUpdateKeyFrames(
           [&](const size_t idx , std::shared_ptr<KeyFrame>& kf_ptr)->void{
               {
                   std::lock_guard<std::mutex> lock(kf_mutex_);
                   key_frame_map_.emplace(idx ,kf_ptr);
               }
               publishKeyframe(kf_ptr);
           }
        );
        lio_pipe_ptr_->start();
        bool is_initial = false;

        io_ptr_->onWait(
           [&]() ->void {
               lio_pipe_ptr_->waitIfDoneQueueToolLarge();
           }
        );
        //
        io_ptr_->onImu([&](const sensor_msgs::msg::Imu& imu_msg, double t_ns) {
           ImuData imu_data;
           toImuData(imu_data, imu_msg, t_ns);
           lio_pipe_ptr_->pushImu(imu_data);
        });
        io_ptr_->onLidar([&](const sensor_msgs::msg::PointCloud2& cloud_msg, double t_ns) {
           //l_count++;
           //std:: cout << "The"<<l_count<<"th lidar data is "<<t_ns<<std::endl;
           if (stop_requested_.load()) {
             return;
          }

           lidar_count_++;

      if (lidar_count_ >= max_lidar_count_) {
          std::cout << "[Frontend] Early stop requested at lidar "
                    << lidar_count_ << std::endl;

          stop_requested_.store(true);
          return;
      }

           lio_pipe_ptr_->onLidarMsg(cloud_msg, t_ns);
        });
        io_ptr_->onGps(
          [&](const sensor_msgs::msg::NavSatFix& msg, double bag_time_ns) {
             //std::cout<< "gps time: "<<bag_time_ns<<std::endl;
             double lat  = static_cast<double>(msg.latitude);
             double lon  = static_cast<double>(msg.longitude);
             double alt  = static_cast<double>(msg.altitude);

             if(!is_initial) {
               auto res= gps_.InitialRTK(lat,lon);

             }
             GPSData gps_data;
             gps_data.t_ = bag_time_ns;
             gps_data.lat_ = lat;
             gps_data.lon_ = lon;
             gps_data.alt_ = alt;
             gps_.Convert2UTM(gps_data);
             //std::cout<< std::setprecision(10)<<"x: "<<gps_data.x_<<"\n";
             //std::cout<< std::setprecision(10)<<"y: "<<gps_data.y_<<"\n";
             //std::cout<< std::setprecision(10)<<"z: "<<gps_data.z_<<"\n";
             gps_.GetPose(gps_data);
             gps_data_map_.emplace(bag_time_ns, gps_data);
          }
       );

        bool ok = io_ptr_->go();
        if (!ok) {
            std::cerr << "Failed rendering bag.\n";
        }

        if (!stop_requested_.load()) {
            lio_pipe_ptr_->notifyEndOfBag();
            std::cout << "End of bag notified.\n";

            lio_pipe_ptr_->stopDrain();
            if (output_kf_) {
                saveCurrentKeyFrames();
            }
            std::cout << "Lio pipe drained/stopped.\n";
        } else {
            // forced stop path
            lio_pipe_ptr_->stopNow();
            saveCurrentKeyFrames();
            std::cout << "Lio pipe force-stopped.\n";
        }
        return ok;
    }
    catch (const std::exception &e) {
        std::cerr << "Frontend::Run exception: " << e.what() << std::endl;
        lio_pipe_ptr_->stopNow();
        return false;
    }
    catch (...) {
        std::cerr << "Frontend::Run unknown exception.\n";
        lio_pipe_ptr_->stopNow();
        return false;
    }

}

void Frontend::stop() {
    bool was_already_stopping = stop_requested_.exchange(true);
    if (was_already_stopping) {
        return;
    }
    lio_pipe_ptr_->stopDrain();

    saveCurrentKeyFrames();
    kf_q_.stop();
}


bool Frontend::loadFrontendConfig() {
    try {
        YAML::Node root = YAML::LoadFile(init_path_);

        if (!root["frontend"]) {
            std::cerr << "Missing 'frontend' section in yaml: " << init_path_ << std::endl;
            return false;
        }

        YAML::Node cfg = root["frontend"];

        // path
        if (cfg["ros_bag_path"]) {
            fe_options_.ros_bag_path_ = cfg["ros_bag_path"].as<std::string>();
        }

        // lio_options
        LioOptions lio_options;
        if (cfg["num_threads"]) {
            lio_options.num_threads_ = cfg["num_threads"].as<int>();
        }
        if (cfg["slack_sec"]) {
            lio_options.slack_sec_ = cfg["slack_sec"].as<double>();
        }
        if (cfg["down_sample_ratio"]) {
            lio_options.down_sample_ratio_ = cfg["down_sample_ratio"].as<double>();
        }
        if (cfg["down_sample"]) {
            lio_options.down_sample_ = cfg["down_sample"].as<bool>();
        }
        if (cfg["out_cloud_dir"]) {
            lio_options.out_cloud_dir_ = cfg["out_cloud_dir"].as<std::string>();
        }
        if (cfg["out_kf_info_path"]) {
            lio_options.out_kf_info_path_ = cfg["out_kf_info_path"].as<std::string>();
        }

        // io_options
        RosIoOffline::IoOptions io_options;
        if (cfg["print_topics"]) {
            io_options.print_topics_ = cfg["print_topics"].as<bool>();
        }

        // lo_options
        LidarOdometry::LoOption lo_options;
        if (cfg["kf_distance"]) {
            lo_options.kf_distance_ = cfg["kf_distance"].as<double>();
        }

        // ndt_options
        IncNDTOptions inc_options;
        if (cfg["max_iterations"]) {
            inc_options.max_iterations_ = cfg["max_iterations"].as<int>();
        }
        if (cfg["voxel_size"]) {
            inc_options.voxel_size_ = cfg["voxel_size"].as<double>();
        }
        if (cfg["inv_voxel_size"]) {
            inc_options.inv_voxel_size_ = cfg["inv_voxel_size"].as<double>();
        }
        if (cfg["min_effective_pts"]) {
            inc_options.min_effective_pts_ = cfg["min_effective_pts"].as<int>();
        }
        if (cfg["min_pts_in_voxel"]) {
            inc_options.min_pts_in_voxel_ = cfg["min_pts_in_voxel"].as<int>();
        }
        if (cfg["max_pts_in_voxel"]) {
            inc_options.max_pts_in_voxel_ = cfg["max_pts_in_voxel"].as<int>();
        }
        if (cfg["eps"]) {
            inc_options.eps_ = cfg["eps"].as<double>();
        }
        if (cfg["res_outlier_threshold"]) {
            inc_options.res_outlier_threshold_ = cfg["res_outlier_threshold"].as<double>();
        }
        if (cfg["capacity"]) {
            inc_options.capacity_ = cfg["capacity"].as<size_t>();
        }
        if (cfg["nearby_type"]) {
            std::string nearby_type_str = cfg["nearby_type"].as<std::string>();
            if (nearby_type_str == "center") {
                inc_options.nearby_type_ = NearbyType::CENTER;
            } else if (nearby_type_str == "nearby6") {
                inc_options.nearby_type_ = NearbyType::NEARBY6;
            } else {
                std::cerr << "Unknown nearby_type: " << nearby_type_str
                          << ", fallback to CENTER" << std::endl;
                inc_options.nearby_type_ = NearbyType::CENTER;
            }
        }

        lo_options.inc_opt_ = inc_options;

        // eskf options
        ESKF::ESKFOptions eskf_options;
        if (cfg["imu_dt"]) {
            eskf_options.imu_dt_ = cfg["imu_dt"].as<double>();
        }
        if (cfg["gyro_var"]) {
            eskf_options.gyro_var_ = cfg["gyro_var"].as<double>();
        }
        if (cfg["acc_var"]) {
            eskf_options.acc_var_ = cfg["acc_var"].as<double>();
        }
        if (cfg["bias_gyro_var"]) {
            eskf_options.bias_gyro_var_ = cfg["bias_gyro_var"].as<double>();
        }
        if (cfg["bias_acc_var"]) {
            eskf_options.bias_acc_var_ = cfg["bias_acc_var"].as<double>();
        }
        if (cfg["pos_noise"]) {
            eskf_options.pos_noise_ = cfg["pos_noise"].as<double>();
        }
        if (cfg["height_noise"]) {
            eskf_options.height_noise_ = cfg["height_noise"].as<double>();
        }
        if (cfg["ang_noise"]) {
            eskf_options.ang_noise_ = cfg["ang_noise"].as<double>() * (M_PI / 180.0);
        }

        lio_options.eskf_options_ = eskf_options;
        lio_options.lo_options_ = lo_options;

        fe_options_.io_options_ = io_options;
        fe_options_.lio_options_ = lio_options;

        return true;
    }
    catch (const YAML::BadFile& e) {
        std::cerr << "Failed to open yaml file: " << init_path_ << "\n";
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error in file: " << init_path_ << "\n";
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cerr << "Unexpected error while loading config: " << e.what() << std::endl;
        return false;
    }
}

//-------------------Publish--------------------------------//
void Frontend::publishCloudPath(const std::string &new_path) {
  std::lock_guard<std::mutex> lock(kf_mutex_);
  path_queue_.push(new_path);
}

bool Frontend::hasKfCloudPath() const {
   std::lock_guard<std::mutex>lock(kf_mutex_);
   return path_queue_.size() > 0 ;
}

std::string Frontend::takeNextPath() {
   std::lock_guard<std::mutex>lock(kf_mutex_);
   if (path_queue_.empty()) {
     return " ";
   }
   std::string path = path_queue_.front();

    path_queue_.pop();
   return path;
}

void Frontend::publishKeyframe(const std::shared_ptr<KeyFrame> kf_ptr) {
  kf_q_.push(kf_ptr);
}

std::shared_ptr<KeyFrame> Frontend::takeNextKeyFrame() {
   std::shared_ptr<KeyFrame>kf_ptr= make_shared<KeyFrame>();
   kf_q_.wait_and_pop(kf_ptr);
   return kf_ptr;
}

std::string Frontend::getOutKfPath() {
  std::cout<<"getOutKfPath"<<fe_options_.lio_options_.out_kf_info_path_<<std::endl;
  return fe_options_.lio_options_.out_kf_info_path_;
}

void Frontend::saveCurrentKeyFrames() {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    bool first_gps = true;
    Vec3d origin = Vec3d::Zero();

    for (auto& it : key_frame_map_) {
        auto& kf = it.second;

        Interpolate_gps(
            kf->time_,
            kf,
            gps_data_map_);

        if (first_gps) {
            origin = kf->rtk_pose_.translation();
            first_gps = false;
        }

        kf->rtk_pose_.translation() -= origin;
    }

    if (output_kf_) {
        writeKeyFramesToFile(
            fe_options_.lio_options_.out_kf_info_path_,
            key_frame_map_);
    }

    std::cout << "[Frontend] Saved current keyframes: "
              << key_frame_map_.size()
              << std::endl;
}

void Frontend::requestStopAndSave() {
    bool old = stop_requested_.exchange(true);
    if (old) return;

    std::cout << "[Frontend] Stop requested, saving current KFs...\n";

    if (lio_pipe_ptr_) {
        lio_pipe_ptr_->stopNow();   // or stopDrain() if you prefer waiting
    }

    if (output_kf_) {
        saveCurrentKeyFrames();
    }

    kf_q_.stop();
}


