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
#include <cmath>
#include <stdexcept>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "../../include/KeyFrame.h"

using namespace std;
Frontend::Frontend(const std::string &init_path, bool localization_test): localization_test_(localization_test), init_path_(init_path) {
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
          if (!pc2ToPointCloudXYZIT(msg_ptr, pc, options.down_sample_ratio_,
                                    options.down_sample_ && !options.use_imu_, true,
                                    options.exclude_lidar_x_band_,
                                    static_cast<float>(options.excluded_lidar_x_min_),
                                    static_cast<float>(options.excluded_lidar_x_max_))) {
              throw std::runtime_error("Invalid point cloud layout or point timestamps");
          }
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


        io_ptr_->onWait(
           [&]() ->void {
               if (stop_requested_.load()) throw std::runtime_error("Mapping stop requested");
               lio_pipe_ptr_->waitIfDoneQueueToolLarge();
           }
        );
        //
        io_ptr_->onImu([&](const sensor_msgs::msg::Imu& imu_msg, double t_ns) {
           ImuData imu_data;
           toImuData(imu_data, imu_msg, t_ns, fe_options_.lio_options_.imu_acc_scale_);
           lio_pipe_ptr_->pushImu(imu_data);
        });
        io_ptr_->onLidar([&](const sensor_msgs::msg::PointCloud2& cloud_msg, double t_ns) {
           //l_count++;
           //std:: cout << "The"<<l_count<<"th lidar data is "<<t_ns<<std::endl;
           if (stop_requested_.load()) {
             return;
          }

           lio_pipe_ptr_->onLidarMsg(cloud_msg, t_ns);
        });
        io_ptr_->onGps(
          [&](const sensor_msgs::msg::NavSatFix& msg, double bag_time_ns) {
             if (!std::isfinite(bag_time_ns) || bag_time_ns <= 0) return;
             GPSData gps_data;
             gps_data.t_ = bag_time_ns; // Callback supplies sensor time in seconds.
             gps_data.lat_ = msg.latitude;
             gps_data.lon_ = msg.longitude;
             gps_data.alt_ = msg.altitude;
             gps_data.valid_ = gnssFixUsable(msg, fe_options_.gnss_options_);
             if (gps_data.valid_ && !gps_.initialized_)
                 gps_data.valid_ = gps_.InitialRTK(msg.latitude, msg.longitude);
             if (gps_data.valid_) gps_data.valid_ = gps_.Convert2UTM(gps_data) && gps_.GetPose(gps_data);
             // Keep rejected fixes so interpolation cannot bridge a no-fix interval.
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
        if (lio_pipe_ptr_) lio_pipe_ptr_->stopNow();
        saveCurrentKeyFrames();
        return stop_requested_.load();
    }
    catch (...) {
        std::cerr << "Frontend::Run unknown exception.\n";
        if (lio_pipe_ptr_) lio_pipe_ptr_->stopNow();
        saveCurrentKeyFrames();
        return stop_requested_.load();
    }

}

void Frontend::stop() {
    requestStopAndSave();
}

void Frontend::requestStop() {
    stop_requested_.store(true);
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
        fe_options_.gnss_options_ = loadGnssOptions(root);

        // path
        if (cfg["ros_bag_path"]) {
            fe_options_.ros_bag_path_ = cfg["ros_bag_path"].as<std::string>();
        }

        // lio_options
        LioOptions lio_options;
        if (cfg["use_imu"]) {
            lio_options.use_imu_ = cfg["use_imu"].as<bool>();
        }
        if (cfg["synchronous_preprocessing"]) lio_options.synchronous_preprocessing_ = cfg["synchronous_preprocessing"].as<bool>();
        if (cfg["excluded_lidar_x_band"]) {
            const auto band=cfg["excluded_lidar_x_band"];
            if (!band.IsSequence() || band.size()!=2)
                throw std::invalid_argument("excluded_lidar_x_band must contain [min_x, max_x]");
            lio_options.exclude_lidar_x_band_=true;
            lio_options.excluded_lidar_x_min_=band[0].as<double>();
            lio_options.excluded_lidar_x_max_=band[1].as<double>();
            if (!std::isfinite(lio_options.excluded_lidar_x_min_) ||
                !std::isfinite(lio_options.excluded_lidar_x_max_) ||
                lio_options.excluded_lidar_x_max_<=lio_options.excluded_lidar_x_min_)
                throw std::invalid_argument("excluded_lidar_x_band must be finite and increasing");
            std::cout<<"Excluding LiDAR-frame x in ("<<lio_options.excluded_lidar_x_min_
                     <<", "<<lio_options.excluded_lidar_x_max_<<") m before voxel filtering\n";
        }
        if (cfg["deskew_enabled"]) lio_options.deskew_enabled_ = cfg["deskew_enabled"].as<bool>();
        if (cfg["imu_recovery"]) {
            const auto recovery = cfg["imu_recovery"];
            if (recovery["enabled"]) lio_options.imu_recovery_enabled_ = recovery["enabled"].as<bool>();
            if (recovery["good_scans"]) {
                const int count = recovery["good_scans"].as<int>();
                if (count <= 0) throw std::invalid_argument("imu_recovery.good_scans must be positive");
                lio_options.imu_recovery_good_scans_ = static_cast<size_t>(count);
            }
            if (!lio_options.imu_recovery_good_scans_) throw std::invalid_argument("imu_recovery.good_scans must be positive");
        }
        if (cfg["imu_prediction_log"]) lio_options.imu_prediction_log_ = cfg["imu_prediction_log"].as<bool>();
        if (cfg["imu_prediction_log_path"]) lio_options.imu_prediction_log_path_ = cfg["imu_prediction_log_path"].as<std::string>();
        if (cfg["imu_acc_scale"]) {
            lio_options.imu_acc_scale_ = cfg["imu_acc_scale"].as<double>();
            if (!std::isfinite(lio_options.imu_acc_scale_) || lio_options.imu_acc_scale_ <= 0) {
                throw std::invalid_argument("imu_acc_scale must be finite and positive");
            }
        }
        if (cfg["imu_lidar_extrinsic"]) {
            const auto extrinsic = cfg["imu_lidar_extrinsic"];
            const auto rotation = extrinsic["rotation"];
            const auto translation = extrinsic["translation_m"];
            if (!rotation.IsSequence() || rotation.size() != 9 ||
                !translation.IsSequence() || translation.size() != 3) {
                throw std::invalid_argument("imu_lidar_extrinsic needs a row-major 9-value rotation and 3-value translation_m");
            }
            Mat3d R;
            Vec3d t;
            for (int row = 0; row < 3; ++row) {
                t[row] = translation[row].as<double>();
                for (int col = 0; col < 3; ++col) R(row, col) = rotation[3 * row + col].as<double>();
            }
            lio_options.T_imu_lidar_ = LidarImuFrames::makeExtrinsic(R, t);
        }
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
        io_options.finish_imu_scan_ = lio_options.use_imu_;
        if (cfg["print_topics"]) {
            io_options.print_topics_ = cfg["print_topics"].as<bool>();
        }

        if (cfg["max_lidar_frames"]) {
            io_options.max_lidar_frames_ = cfg["max_lidar_frames"].as<size_t>();
        }

        // lo_options
        LidarOdometry::LoOption lo_options;
        if (cfg["kf_distance"]) {
            lo_options.kf_distance_ = cfg["kf_distance"].as<double>();
        }

        if (cfg["kf_angle_degree"]) {
            lo_options.kf_angle_degree_ = cfg["kf_angle_degree"].as<double>();
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
        if (cfg["imu_route_path"]) eskf_options.out_pos_path_ = cfg["imu_route_path"].as<std::string>();
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

        if (cfg["imu_initialization"]) {
            const auto init = cfg["imu_initialization"];
            auto& options = eskf_options.imu_initialization;
            if (init["duration_sec"]) options.duration_sec = init["duration_sec"].as<double>();
            if (init["min_samples"]) options.min_samples = init["min_samples"].as<size_t>();
            if (init["max_gap_sec"]) options.max_gap_sec = init["max_gap_sec"].as<double>();
            if (init["max_gyro_mean"]) options.max_gyro_mean = init["max_gyro_mean"].as<double>();
            if (init["max_gyro_std"]) options.max_gyro_std = init["max_gyro_std"].as<double>();
            if (init["max_acc_std"]) options.max_acc_std = init["max_acc_std"].as<double>();
            if (init["max_gravity_error"]) options.max_gravity_error = init["max_gravity_error"].as<double>();
        }

        lio_options.eskf_options_ = eskf_options;
        lio_options.lo_options_ = lo_options;

        if(localization_test_) {
            const auto loc=root["localization_test"];
            if(!loc)throw std::invalid_argument("localization_test mode requires its YAML section");
            auto& o=lio_options.localization_;o.enabled=true;
            if(loc["seed_source"])o.seed_source=loc["seed_source"].as<std::string>();
            if(o.seed_source!="motion"&&o.seed_source!="last_accepted"&&o.seed_source!="lo")
                throw std::invalid_argument("localization_test.seed_source must be motion, last_accepted, or lo");
            if(loc["reference_keyframes"])o.reference_keyframes=loc["reference_keyframes"].as<std::string>();
            if(loc["reference_cloud"])o.reference_cloud=loc["reference_cloud"].as<std::string>();
            if(loc["reference_stage"])o.reference_stage=loc["reference_stage"].as<std::string>();
            auto read_pose=[](const YAML::Node& n)->SE3d {
                if(!n.IsSequence()||n.size()!=7)throw std::invalid_argument("Pose requires [x,y,z,qx,qy,qz,qw]");
                double v[7];for(int i=0;i<7;++i){v[i]=n[i].as<double>();if(!std::isfinite(v[i]))throw std::invalid_argument("Nonfinite pose");}
                Eigen::Quaterniond q(v[6],v[3],v[4],v[5]);if(q.norm()<1e-12)throw std::invalid_argument("Zero quaternion");
                return SE3d(q.normalized(),Vec3d(v[0],v[1],v[2]));
            };
            o.initial_pose=read_pose(loc["initial_pose"]);
            if(loc["reference_cloud_pose"])o.reference_cloud_pose=read_pose(loc["reference_cloud_pose"]);
            auto frame_value=[&](const char* key,size_t& value){if(loc[key]){const auto v=loc[key].as<long long>();if(v<0)throw std::invalid_argument("Negative frame index");value=size_t(v);}};
            frame_value("start_frame",o.start_frame);frame_value("end_frame",o.end_frame);
            auto number=[&](const char* key,double& value){if(loc[key])value=loc[key].as<double>();};
            number("map_voxel_m",o.map_voxel_m);number("local_radius_m",o.local_radius_m);number("refresh_distance_m",o.refresh_distance_m);
            number("min_coverage",o.min_coverage);number("max_inlier_rmse_m",o.max_inlier_rmse_m);number("max_correction_m",o.max_correction_m);number("max_correction_deg",o.max_correction_deg);
            frame_value("min_matches",o.min_matches);
            namespace fs=std::filesystem;
            if(lio_options.out_cloud_dir_.empty())throw std::invalid_argument("Localization needs an isolated output directory");
            const auto output=fs::weakly_canonical(lio_options.out_cloud_dir_);
            if(fs::exists(output)&&(!fs::is_directory(output)||!fs::is_empty(output)))throw std::runtime_error("Localization output directory must be new or empty");
            auto inside=[&](const std::string& path){auto relative=fs::weakly_canonical(path).lexically_relative(output);return !path.empty()&&!relative.empty()&&*relative.begin()!="..";};
            if(!inside(lio_options.out_kf_info_path_)||!inside(lio_options.eskf_options_.out_pos_path_)||(!lio_options.imu_prediction_log_path_.empty()&&!inside(lio_options.imu_prediction_log_path_)))throw std::invalid_argument("All localization outputs must be inside out_cloud_dir");
            fs::create_directories(output);
            o.log_path=(output/"localization.csv").string();
            if(o.end_frame)io_options.max_lidar_frames_=o.end_frame;
            output_kf_=true;
        }

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
   if (!kf_q_.try_pop(kf_ptr)) return nullptr;
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

    if (key_frame_map_.empty()) return;

    if(localization_test_) {
        // Preserve reference-map coordinates. New-visit GNSS must not re-align them.
        writeKeyFramesToFile(fe_options_.lio_options_.out_kf_info_path_,key_frame_map_);
        std::cout<<"[Localization test] Saved accepted scan poses: "<<key_frame_map_.size()<<"\n";
        return;
    }

    std::map<size_t, std::shared_ptr<KeyFrame>> snapshot;
    for (const auto& it : key_frame_map_) {
        if (it.second) snapshot.emplace(it.first, std::make_shared<KeyFrame>(*it.second));
    }
    for (auto& it : snapshot) {
        auto& kf = it.second;

        kf->rtk_valid_ = Interpolate_gps(
            kf->time_,
            kf,
            gps_data_map_, fe_options_.gnss_options_.max_gap_sec);
        kf->rtk_inlier_ = kf->rtk_valid_;
        if (!kf->rtk_valid_) {
            kf->rtk_pose_ = SE3d();
            kf->rtk_heading_valid_ = false;
            continue;
        }

        if (first_gps) {
            origin = kf->rtk_pose_.translation();
            first_gps = false;
        }

        kf->rtk_pose_.translation() -= origin;
    }

    if (output_kf_) {
        writeKeyFramesToFile(
            fe_options_.lio_options_.out_kf_info_path_,
            snapshot);
    }

    std::cout << "[Frontend] Saved current keyframes: "
              << key_frame_map_.size()
              << std::endl;
}

void Frontend::requestStopAndSave() {
    requestStop();

    std::cout << "[Frontend] Stop requested, saving current KFs...\n";

    if (lio_pipe_ptr_) {
        lio_pipe_ptr_->stopNow();   // or stopDrain() if you prefer waiting
    }

    if (output_kf_) {
        saveCurrentKeyFrames();
    }

    kf_q_.stop();
}


