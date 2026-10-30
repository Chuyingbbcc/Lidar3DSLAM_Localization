//
// Created by chuchu on 6/1/26.
//


#include "../../include/Slam/Backend.h"

#include "KeyFrame.h"
#include "Slam/Backend.h"
#include "DataType.h"
#include <iostream>
#include <memory>
#include <map>
#include <cmath>
#include <algorithm>   // std::sort, std::min, std::max
#include <set>         // std::set
#include <utility>     // std::pair
#include <fstream>
#include <iomanip>
#include <cstdio>
#include <stdexcept>
#include <sstream>

#include  "Submap.h"
#include "Align.h"
#include "common.h"
#include <yaml-cpp/yaml.h>
#include "RouteAlign.h"
#include "io/ros_io_offline.h"
#include "Slam/LoopVerification.h"
#include <Eigen/Core>

#include "../../include/common.h"


bool Backend::loadBackendConfig(const std::string &init_path) {
  try {
        YAML::Node root = YAML::LoadFile(init_path);

        if (!root["backend"]) {
            std::cerr << "Missing 'backend' section in yaml: " << init_path << std::endl;
            return false;
        }

        YAML::Node cfg = root["backend"];
        backend_config_.gnss_options_ = loadGnssOptions(root);
        if(cfg["use_gnss"]) backend_config_.use_gnss_=cfg["use_gnss"].as<bool>();
        if(cfg["gnss_xy_only"]) backend_config_.gnss_xy_only_=cfg["gnss_xy_only"].as<bool>();
        if(cfg["gnss_local_to_global"]) backend_config_.gnss_local_to_global_=cfg["gnss_local_to_global"].as<bool>();
        auto positive=[&](const char* key,double& value){if(cfg[key])value=cfg[key].as<double>();if(!std::isfinite(value)||value<=0)throw std::invalid_argument(std::string("Invalid backend weight: ")+key);};
        positive("lio_information",backend_config_.lio_information_);
        positive("loop_information",backend_config_.loop_information_);
        positive("gnss_huber_delta",backend_config_.gnss_huber_delta_);
        if(cfg["gnss_information_xy"]) backend_config_.gnss_information_xy_=cfg["gnss_information_xy"].as<double>();
        if(cfg["gnss_min_interval_sec"]) backend_config_.gnss_min_interval_sec_=cfg["gnss_min_interval_sec"].as<double>();
        if(cfg["fixed_loop_path"]) backend_config_.fixed_loop_path_=cfg["fixed_loop_path"].as<std::string>();
        if(cfg["gnss_diagnostics_path"]) backend_config_.gnss_diagnostics_path_=cfg["gnss_diagnostics_path"].as<std::string>();
        if(backend_config_.gnss_xy_only_ &&
           (!std::isfinite(backend_config_.gnss_information_xy_) || backend_config_.gnss_information_xy_<=0 ||
            !std::isfinite(backend_config_.gnss_min_interval_sec_) || backend_config_.gnss_min_interval_sec_<=0 ||
            (!backend_config_.gnss_local_to_global_ && backend_config_.fixed_loop_path_.empty()))) throw std::invalid_argument("Invalid XY-only GNSS settings");
        if(cfg["loop_candidate_log_path"])backend_config_.loop_candidate_log_path_=cfg["loop_candidate_log_path"].as<std::string>();
        if(cfg["loop"]) {
            const auto loop=cfg["loop"];auto& o=backend_config_.loop_closure_options_;
            auto number=[&](const char* key,double& value){if(loop[key])value=loop[key].as<double>();if(!std::isfinite(value)||value<=0)throw std::invalid_argument(std::string("Invalid loop option: ")+key);};
            number("min_time_sec",o.min_time_sec_);number("min_travel_m",o.min_travel_m_);number("query_interval_sec",o.query_interval_sec_);number("history_group_sec",o.history_group_sec_);
            number("window_half_sec",o.window_half_sec_);number("duplicate_sec",o.duplicate_sec_);number("xy_radius_m",o.kf_xy_thresh_);number("z_radius_m",o.kf_z_thresh_);
            number("max_correction_m",o.max_correction_m_);number("max_correction_deg",o.max_correction_deg_);number("point_distance_m",o.point_distance_m_);number("min_bidirectional_overlap",o.min_bidirectional_overlap_);number("max_point_rmse_m",o.max_point_rmse_m_);
            if(loop["max_candidates"])o.max_candidates_=loop["max_candidates"].as<size_t>();
            if(loop["min_matched_points"])o.min_matched_points_=loop["min_matched_points"].as<size_t>();
            if(loop["voxel_sizes_m"])o.voxel_sizes_=loop["voxel_sizes_m"].as<std::vector<double>>();
            if(loop["full_3d"])o.full_3d_=loop["full_3d"].as<bool>();
            if(loop["consecutive_verification"])o.consecutive_verification_=loop["consecutive_verification"].as<bool>();
            if(loop["consecutive_frames"])o.consecutive_frames_=loop["consecutive_frames"].as<size_t>();
            if(loop["consecutive_min_matches"])o.consecutive_min_matches_=loop["consecutive_min_matches"].as<size_t>();
            number("consecutive_max_gap_sec",o.consecutive_max_gap_sec_);
            number("consensus_translation_m",o.consensus_translation_m_);
            number("consensus_rotation_deg",o.consensus_rotation_deg_);
            if(loop["wall_aware"]) {
                const auto wall=loop["wall_aware"];auto& w=o.wall_aware_;
                if(wall["enabled"])w.enabled=wall["enabled"].as<bool>();
                auto wall_number=[&](const char* key,double& value){if(wall[key])value=wall[key].as<double>();if(!std::isfinite(value)||value<=0)throw std::invalid_argument(std::string("Invalid wall-aware loop option: ")+key);};
                wall_number("voxel_size_m",w.voxel_size_m);
                wall_number("max_planarity_ratio",w.max_planarity_ratio);
                wall_number("max_normal_z",w.max_normal_z);
                wall_number("max_correspondence_m",w.max_correspondence_m);
                wall_number("huber_delta_m",w.huber_delta_m);
                wall_number("observable_eigen_ratio",w.observable_eigen_ratio);
                wall_number("weak_information_ratio",w.weak_information_ratio);
                if(wall["optimize_z"])w.optimize_z=wall["optimize_z"].as<bool>();
                wall_number("min_horizontal_normal_z",w.min_horizontal_normal_z);
                wall_number("floor_max_correspondence_m",w.floor_max_correspondence_m);
                wall_number("floor_huber_delta_m",w.floor_huber_delta_m);
                wall_number("floor_information_ratio",w.floor_information_ratio);
                if(wall["min_points_per_voxel"])w.min_points_per_voxel=wall["min_points_per_voxel"].as<size_t>();
                if(wall["max_iterations"])w.max_iterations=wall["max_iterations"].as<int>();
                if(wall["min_correspondences"])w.min_correspondences=wall["min_correspondences"].as<size_t>();
                if(wall["min_observable_modes"])w.min_observable_modes=wall["min_observable_modes"].as<int>();
                if(wall["floor_min_correspondences"])w.floor_min_correspondences=wall["floor_min_correspondences"].as<size_t>();
                if(!w.min_points_per_voxel||w.max_iterations<1||!w.min_correspondences||w.min_observable_modes<1||w.min_observable_modes>3||
                   !w.floor_min_correspondences||w.max_planarity_ratio>=1||w.max_normal_z>=1||w.min_horizontal_normal_z>=1||
                   w.observable_eigen_ratio>=1||w.weak_information_ratio>=1||w.floor_information_ratio>=1)
                    throw std::invalid_argument("Invalid wall-aware loop geometry thresholds");
            }
            if(o.consecutive_verification_ && (o.consecutive_frames_<2 || o.consecutive_frames_>10 || !o.consecutive_min_matches_))throw std::invalid_argument("Invalid loop confirmation counts");
            if(!o.max_candidates_||!o.min_matched_points_||o.min_bidirectional_overlap_>1||o.voxel_sizes_.empty())throw std::invalid_argument("Invalid loop verification options");
            double previous=INFINITY;for(double size:o.voxel_sizes_){if(!std::isfinite(size)||size<=0||size>=previous)throw std::invalid_argument("Loop voxel sizes must decrease and be positive");previous=size;}
        }
        if (root["frontend"] && root["frontend"]["ros_bag_path"])
            backend_config_.gnss_bag_path_ = root["frontend"]["ros_bag_path"].as<std::string>();

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
      backend_config_.inc_ndt_options_ = inc_options;
        // path
        if (cfg["sub_map_size"]) {
            backend_config_.submap_size_ = cfg["sub_map_size"].as<size_t>();
        }

        if (cfg["in_cloud_dir"]) {
            backend_config_.in_cloud_dir_ = cfg["in_cloud_dir"].as<std::string>();
        }
        if (cfg["in_kf_info_path"]) {
           backend_config_.in_kf_info_path_ = cfg["in_kf_info_path"].as<std::string>();
        }
        return true;
    }
    catch (const YAML::BadFile& e) {
        std::cerr << "Failed to open yaml file: " << init_path << "\n";
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error in file: " << init_path << "\n";
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cerr << "Unexpected error while loading config: " << e.what() << std::endl;
        return false;
    }
}

Backend::Backend(const std::string &init_path, std::function<bool()> stop_requested)
    : stop_requested_(std::move(stop_requested)) {
  if (!loadBackendConfig(init_path)) throw std::runtime_error("Invalid backend configuration: " + init_path);
  submap_manager_ptr_ = std::make_unique<SubmapManager>(backend_config_.submap_size_);
}

void Backend::addKeyFrame(std::shared_ptr<KeyFrame> kf) {
  key_frames_.emplace(kf->id_, kf);
  //add this kf to submap
  processNewKeyFrame(kf);
}

void Backend::processNewKeyFrame(std::shared_ptr<KeyFrame> kf) {
  submap_manager_ptr_->addKfToSubmap(kf);
}

void Backend::buildSubmaps() {
  loadKeyFrames(backend_config_.in_kf_info_path_, key_frames_, true);
  if (!key_frames_.empty() && backend_config_.use_gnss_) {
    std::map<double, GPSData> coverage;
    bool audited = false;
    if (!backend_config_.gnss_bag_path_.empty()) {
      try {
        RosIoOffline reader(backend_config_.gnss_bag_path_);
        reader.io_options_.topics = {"/gps/fix"};
        reader.io_options_.print_topics_ = false;
        double latest = 0;
        for (const auto& entry : key_frames_) latest = std::max(latest, entry.second->time_);
        reader.io_options_.max_gps_time_ = latest + backend_config_.gnss_options_.max_gap_sec;
        reader.onGps([&](const sensor_msgs::msg::NavSatFix& msg, double time) {
          if (!std::isfinite(time) || time <= 0) return;
          GPSData sample;
          sample.t_ = time;
          sample.valid_ = gnssFixUsable(msg, backend_config_.gnss_options_);
          coverage.emplace(time, sample);
        });
        reader.onWait([&] { if (stopped()) throw std::runtime_error("GNSS audit interrupted"); });
        audited = reader.go();
      } catch (const std::exception& error) {
        std::cerr << "[GNSS] Coverage audit failed: " << error.what() << "\n";
      }
    }
    if (!audited) {
      coverage.clear(); // Fail closed: retain odometry, disable unverifiable GNSS edges.
      std::cerr << "[GNSS] Source coverage unavailable; disabling GNSS constraints\n";
    }
    const size_t rejected = applyGnssCoverage(key_frames_, coverage, backend_config_.gnss_options_.max_gap_sec);
    std::cout << "[GNSS] Coverage audit rejected " << rejected << "/" << key_frames_.size()
              << " keyframes (max interpolation gap " << backend_config_.gnss_options_.max_gap_sec << " s)\n";
  }
  size_t last_id = 0;
  bool first = true;
  for (auto it : key_frames_) {
     if (first) {
        last_id = it.first;
        continue;
     }
     if (it.first - last_id !=1) {
         std::cout<<"it's not ordered"<<std::endl;
     }
  }
  //std::cout<< "be kfs size: " <<key_frames_.size() << std::endl;
  for (auto it : key_frames_) {
    addKeyFrame(it.second);
  }
}

void Backend::optimizeOneSubmapLevel10(Submap& submap) {
  if(submap.key_frames_.empty()) {
    return;
  }
  IncNDT fixed_ndt(backend_config_.inc_ndt_options_);
  //create fixed world
  for (auto& kf: submap.key_frames_) {
    //need load cloud one more time
    std::shared_ptr<PointCloud>cloud = std::make_shared<PointCloud>();
    loadPointCloudFromFile(kf->cloud_path_, cloud);
    transformCloud(cloud, kf->fst_opti_pose_);
    fixed_ndt.AddCloud(cloud);
  }

  //refine each kf against fixed map
  for (auto& kf: submap.key_frames_) {
    SE3d init_pose= kf->fst_opti_pose_;
    NdtEval before_eva = fixed_ndt.computeScore(kf->fst_opti_pose_);
    double score_before = before_eva.score_;
    fixed_ndt.SetSourceCloud(kf->cloud_ptr_);
    bool ok  = fixed_ndt.Align(init_pose);
    NdtEval after_eva = fixed_ndt.computeScore(init_pose);
    double score_after = after_eva.score_;
    bool improved = std::isfinite(score_before) &&
    std::isfinite(score_after) &&
    score_after < score_before * 0.98;

    if (ok && improved) {
      kf->scd_opti_pose_ = init_pose;
    }
    else {
      kf->scd_opti_pose_ = kf->fst_opti_pose_;
      std::cout << "keep use old pose" <<std::endl;
      std::cout<< "Score before: " << score_before << std::endl;
      std::cout << "Score after: " << score_after << std::endl;
      std::cout << "------------------------------------"<<std::endl;
    }
    SE3d delta = kf->fst_opti_pose_.inverse() * kf->scd_opti_pose_;
    Vec3d dt = delta.translation();

    double trans_m = dt.norm();

    // total 3D rotation
    double rot3d_deg =
        delta.so3().log().norm() * 180.0 / M_PI;

    // yaw only
    Mat3d R = delta.rotationMatrix();

    double yaw_deg =
        std::atan2(
            R(1,0),
            R(0,0))
        * 180.0 / M_PI;

    std::cout
        << "[Lv1] KF "
        << kf->id_
        << "\n"
        << "  dxyz      = "
        << dt.transpose()
        << "\n"
        << "  trans     = "
        << trans_m
        << " m\n"
        << "  yaw       = "
        << yaw_deg
        << " deg\n"
        << "  rot3d     = "
        << rot3d_deg
        << " deg\n"
        << "------------------------------------"<<std::endl;
  }
  // submap.pose_init_ = submap.key_frames_.front()->fst_opti_pose_;
  // submap.pose_optimized_ = submap.pose_init_;
}

void Backend::runSubmapInsideOptimization() {
  auto& submaps =  submap_manager_ptr_ ->getSubmaps();
  for (auto& submap : submaps) {
    optimizeOneSubmapLevel10(submap);
  }
}

//Todo: might delete later

// void Backend::runLevel2Optimization() {
//     std::vector<PoseGraphOptimizer::Node>node_vect;
//     buildSubmapGraphNodes(node_vect);
//     std::cout
//     << "\n========== Lv2 Input ==========\n";
//
//     for (const auto& node : node_vect) {
//
//         std::cout
//             << "[Node] "
//             << node.id_
//             << " pose="
//             << node.pose_init_.translation().transpose();
//
//         if (node.has_gps_) {
//             double diff =
//                 (node.pose_init_.translation() -
//                  node.gps_pos_).norm();
//
//             std::cout
//                 << " gps="
//                 << node.gps_pos_.transpose()
//                 << " diff="
//                 << diff;
//         }
//
//         std::cout << std::endl;
//     }
//     PoseGraphOptimizer optimizer;
//     optimizer.setNodes(node_vect);
//     bool ok = optimizer.optimize(20);
//     if(!ok) {
//         std::cout << "[Lv2] optimization failed, use passthrough.\n";
//         applyLevel20Correction();
//         return;
//     }
//     std::map<size_t,SE3d> opt_poses;
//     optimizer.getOptimizedPoses(opt_poses);
//     std::cout<< "\n========== Lv2 Result ==========\n";
//     //update back to the submap
//     for (auto& sm : submap_manager_ptr_->getSubmaps()) {
//         auto it = opt_poses.find(sm.id_);
//         if (it != opt_poses.end()) {
//             sm.pose_optimized_ = it->second;
//             Vec3d t_init = sm.pose_init_.translation();
//             Vec3d t_opt  = sm.pose_optimized_.translation();
//
//             Vec3d move = t_opt - t_init;
//
//             SE3d rel = sm.pose_init_.inverse() * sm.pose_optimized_;
//
//             double yaw_deg =
//                 std::atan2(
//                     rel.rotationMatrix()(1, 0),
//                     rel.rotationMatrix()(0, 0))
//                 * 180.0 / M_PI;
//
//             std::cout << "[Lv2] Submap " << sm.id_
//                       << "\n init = " << t_init.transpose()
//                       << "\n opt  = " << t_opt.transpose()
//                       << "\n move = " << move.transpose()
//                       << "\n move_xy = " << move.head<2>().norm()
//                       << "\n move_xyz = " << move.norm()
//                       << "\n rel_yaw = " << yaw_deg << " deg"
//                       << "\n----------------------\n";
//             if (move.head<2>().norm() > 10.0 ||
//     std::abs(yaw_deg) > 5.0) {
//                 std::cout<<"[Lv2 warning] change too large!"<<std::endl;
//             }
//             size_t mid = sm.key_frames_.size()/2;
//             auto sm_gps_pos = sm.key_frames_[mid]->rtk_pose_.translation();
//             double before =
//              (sm.pose_init_.translation() - sm_gps_pos).norm();
//
//             double after =
//                 (sm.pose_optimized_.translation() - sm_gps_pos).norm();
//
//             std::cout << "[GPS Check] Submap " << sm.id_
//                       << " before=" << before
//                       << " after=" << after
//                       << std::endl;
//
//         }
//     }
//     applyLevel20Correction();
// }

void Backend::runKfRtkOptimization() {
    if(!backend_config_.use_gnss_ || backend_config_.gnss_xy_only_) {
        for(auto& entry:key_frames_) {
            if(!backend_config_.use_gnss_) entry.second->lidar_pose_neu_=entry.second->lidar_pose_;
            entry.second->fst_opti_pose_=entry.second->lidar_pose_neu_;
            entry.second->scd_opti_pose_=entry.second->lidar_pose_neu_;
        }
        return;
    }
    if (stopped()) return;
    // A failed or incomplete solve must retain the current aligned input.
    for (const auto& entry : key_frames_) {
        if (!entry.second) continue;
        entry.second->fst_opti_pose_ = entry.second->lidar_pose_neu_;
        entry.second->scd_opti_pose_ = entry.second->lidar_pose_neu_;
    }
    std::vector<PoseGraphOptimizer::Node>node_vect;
    std::vector<PoseGraphOptimizer::Edge>edge_vect;

    if (stopped()) return;
    buildKfGraphNodesAndEdges(node_vect, edge_vect);
    std::cout
    << "\n========== Lv2 Input ==========\n"
    << "There are: " << node_vect.size() << " nodes\n";
    Mat3d gps_info = Mat3d::Zero();
    gps_info(0,0) = 0.001;
    gps_info(1,1) = 0.001;
    gps_info(2,2) = 1000.0;
    PoseGraphOptimizer optimizer(OptimizationStage::KF_RTK_OPTI);
    optimizer.setNodes(node_vect);
    optimizer.setEdges(edge_vect);
    optimizer.setStopRequested(stop_requested_);
    optimizer.setGpsInfo(gps_info);
    bool ok = optimizer.optimize(20);
    if(!ok) {
        std::cout << "[Lv2] optimization failed, use passthrough.\n";
        return;
    }
    std::map<size_t,SE3d> opt_poses;
    optimizer.getOptimizedPoses(opt_poses);
    std::cout<< "\n========== Lv2 Result ==========\n";
    //update back to Kfs
    for (auto& kf_it : key_frames_) {
        auto& kf = kf_it.second;
        if (!kf) continue;
        auto it = opt_poses.find(kf->id_);
        if (it != opt_poses.end() && it->second.matrix().allFinite()) {
            kf->fst_opti_pose_ = it->second;
            kf->scd_opti_pose_ = kf->fst_opti_pose_;

            Vec3d t_init = kf->lidar_pose_neu_.translation();
            Vec3d t_opt  = kf->fst_opti_pose_.translation();

            Vec3d move = t_opt - t_init;

            SE3d rel = kf->lidar_pose_neu_.inverse() * kf->fst_opti_pose_;

            double yaw_deg =
                std::atan2(
                    rel.rotationMatrix()(1, 0),
                    rel.rotationMatrix()(0, 0))
                * 180.0 / M_PI;

            std::cout << "[Lv2] KeyFrame: " << kf->id_
                      << "\n move = " << move.transpose()
                      << "\n move_xy = " << move.head<2>().norm()
                      << "\n move_xyz = " << move.norm()
                      << "\n rel_yaw = " << yaw_deg << " deg"
                      << "\n----------------------\n";
            if (move.head<2>().norm() > 10.0 ||
    std::abs(yaw_deg) > 5.0) {
                std::cout<<"[Lv2 warning] change too large!"<<std::endl;
            }
        }
    }
}
void Backend::applyLevel20Correction() {
   auto& submaps = submap_manager_ptr_->getSubmaps();
   for (auto& sm : submaps) {
       if (sm.key_frames_.empty()) {
           continue;
       }
       SE3d delta = sm.pose_optimized_ * sm.pose_init_.inverse();
       //debug
    //    Vec3d dt = delta.translation();
    //
    //    double trans_xy =
    //        std::sqrt(
    //            dt.x() * dt.x() +
    //            dt.y() * dt.y());
    //
    //    double trans_xyz =
    //        dt.norm();
    //
    //    double yaw_deg =
    //        std::atan2(
    //            delta.rotationMatrix()(1,0),
    //            delta.rotationMatrix()(0,0))
    //        * 180.0 / M_PI;
    //
    //    double rot3d_deg =
    //        delta.so3().log().norm()
    //        * 180.0 / M_PI;
    //
    //    std::cout
    // << "[Lv2] Submap "
    // << sm.id_
    // << "\n"
    // << "  dxyz      = "
    // << dt.transpose()
    // << "\n"
    // << "  trans_xy  = "
    // << trans_xy
    // << " m\n"
    // << "  trans_xyz = "
    // << trans_xyz
    // << " m\n"
    // << "  yaw       = "
    // << yaw_deg
    // << " deg\n"
    // << "  rot3d     = "
    // << rot3d_deg
    // << " deg\n"
    // << "-----------------------------------"
    // << std::endl;

       for (auto& kf : sm.key_frames_) {
           kf->scd_opti_pose_ = delta * kf->fst_opti_pose_;
       }
       // Vec3d t_init =
       //      sm.pose_init_.translation();
       //
       // Vec3d t_opt =
       //     sm.pose_optimized_.translation();
       //
       // Vec3d dt =
       //     t_opt - t_init;
       //
       // for (auto& kf : sm.key_frames_) {
       //     SE3d corrected = kf->fst_opti_pose_;
       //     corrected.translation() += dt;
       //
       //     kf->scd_opti_pose_ = corrected;
       // }
   }
}



void Backend::buildKfGraphNodesAndEdges(std::vector<PoseGraphOptimizer::Node>& node_vect, std::vector<PoseGraphOptimizer::Edge>& edge_vect)
 {
    if (key_frames_.empty()) {
        return;
    }
    Mat6d odom_info = Mat6d::Zero();

    odom_info(0,0) = 1000.0;
    odom_info(1,1) = 1000.0;
    odom_info(2,2) = 0.01;

    odom_info(3,3) = 1000.0;
    odom_info(4,4) = 1000.0;
    odom_info(5,5) = 1000.0;
    std::shared_ptr<KeyFrame> previous;
    for (const auto& entry : key_frames_) {
        const auto& kf = entry.second;
        if (!kf || !kf->lidar_pose_neu_.matrix().allFinite()) continue;
        PoseGraphOptimizer::Node node;
        node.id_ = kf->id_;
        node.t_ = kf->time_;
        node.pose_init_ = kf->lidar_pose_neu_;
        node.has_gps_ = kf->rtk_valid_ && kf->rtk_pose_.translation().allFinite();
        if (node.has_gps_) node.gps_pos_ = kf->rtk_pose_.translation();
        node_vect.push_back(node);
        if (previous) {
            PoseGraphOptimizer::Edge edge;
            edge.id_i_ = previous->id_;
            edge.id_j_ = kf->id_;
            edge.T_i_j_ = previous->lidar_pose_neu_.inverse() * kf->lidar_pose_neu_;
            edge.info_ = odom_info;
            edge_vect.push_back(edge);
        }
        previous = kf;
    }
}

void Backend::buildSubmapGraphNodes(std::vector<PoseGraphOptimizer::Node> &node_vect) {
  for (auto& sm : submap_manager_ptr_ ->getSubmaps()) {
    if (sm.key_frames_.empty()) {
       continue;
    }
    PoseGraphOptimizer::Node node;
    node.id_ = sm.id_;
    node.t_ = sm.key_frames_.front()->time_;
    size_t mid = sm.key_frames_.size()/2;
    node.pose_init_ = sm.key_frames_[mid]->fst_opti_pose_;
    node.gps_pos_ = sm.key_frames_[mid]->rtk_pose_.translation();
    // Vec3d  gps_pos;
    // if(computeSubmapRtk(sm, gps_pos)) {
    //     node.has_gps_ = false;
    //     node.gps_pos_ = gps_pos;
    //
    //     std::cout << "[Lv2 GPS] Submap "
    //          << sm.id_
    //          << " init="
    //          << node.pose_init_.translation().transpose()
    //          << " gps="
    //          << gps_pos.transpose()
    //          << " diff="
    //          << (node.pose_init_.translation() - gps_pos).norm()
    //          << " m"
    //          << std::endl;
    // }
    // else {
    //     node.has_gps_ = false;
    // }
    node_vect.push_back(node);
  }
}

void Backend::neuAlign() {
    if(!backend_config_.use_gnss_) {
        for(auto& entry:key_frames_)entry.second->lidar_pose_neu_=entry.second->lidar_pose_;
        std::cout<<"[Backend] GNSS disabled; retaining LiDAR coordinates\n";
        return;
    }
    std::vector<AlignedPair>ap_vect;
    int count =0;
    for(auto& it : key_frames_) {
      //Todo : nnumber of lidars used for neu estimzte
      if (count >1500) {
        break;
      }
      AlignedPair ap;
      auto& kf = it.second;
      if (!kf || !kf->rtk_valid_ ||
          !kf->rtk_pose_.translation().allFinite() ||
          !kf->lidar_pose_.matrix().allFinite()) continue;
      ap.timestamp_ = kf->time_;
      ap.p_odom_ = (backend_config_.gnss_xy_only_ ? kf->loop_opti_pose_ : kf->lidar_pose_).translation();
      ap.p_rtk_ = kf->rtk_pose_.translation();
      ap_vect.emplace_back(ap);
        count++;
    }
    if (ap_vect.size() < 2) {
        std::cerr << "[RTK] Insufficient valid GPS pairs; retaining LiDAR coordinates.\n";
        for (const auto& entry : key_frames_) {
            if (entry.second) entry.second->lidar_pose_neu_ = entry.second->lidar_pose_;
        }
        return;
    }
    //neu align
      Rigid2D align = RouteAlign::estimateRobustRigid2D(ap_vect, 2, 0.85, 10);
      double z_offset = backend_config_.gnss_xy_only_ ? 0.0 : RouteAlign::estimateMedianZOffset(ap_vect, align);

      Mat3d R_align = RouteAlign::yawToRotation3D(align.yaw_);

      Vec3d t_align(
          align.t_.x(),
          align.t_.y(),
          z_offset);

      auto convert2neu = [&](const SE3d& in_pose) -> SE3d {
          Mat3d R_new = R_align * in_pose.rotationMatrix();
          Vec3d t_new = R_align * in_pose.translation() + t_align;

          return SE3d(R_new, t_new);
      };

      for (auto& it : key_frames_) {
          auto& kf = it.second;
          if (!kf) continue;
          kf->lidar_pose_neu_ = convert2neu(kf->lidar_pose_);
          if(backend_config_.gnss_xy_only_) kf->loop_opti_pose_=convert2neu(kf->loop_opti_pose_);

          auto dist = (kf->lidar_pose_neu_.translation() - kf->rtk_pose_.translation()).norm();
          //std::cout<< "kf: " << kf->id_ << " dist: " << dist << std::endl;
      }
}

void Backend::runCorrectNdt() {
   if (key_frames_.empty()) {
      return;
   }

  IncNDT scd_ndt(backend_config_.inc_ndt_options_);
   //---------------handel fst kf------------//
   auto fst_kf = key_frames_.begin()->second;
   const SE3d T_k_enu = fst_kf->lidar_pose_neu_;
   const SE3d T_k_fst_opti = fst_kf->fst_opti_pose_;
   const SO3d T_init_so3 =T_k_enu.so3();
   const Vec3d T_init_trans = T_k_fst_opti.translation();
   fst_kf->scd_opti_pose_ = SE3d(T_init_so3, T_init_trans);
   scd_ndt.AddCloud(fst_kf->cloud_ptr_);

   //-------------------NDT------------------//
   for (size_t i = 2; i<key_frames_.size(); ++i) {
       auto cur_kf =key_frames_[i];
       if(!cur_kf || !cur_kf->cloud_ptr_) {
          std::cerr<< "kf damage!!"<<std::endl;
       }
       scd_ndt.SetSourceCloud(cur_kf->cloud_ptr_);
       //keep the lio so3, and graph opti trans

       auto prev_kf = key_frames_[i-1];

       //lio_neu pose
       const SE3d& prev_lio_neu = prev_kf->lidar_pose_neu_;
       const SE3d& cur_lio_neu = cur_kf->lidar_pose_neu_;

       const SE3d& relative_motion = prev_lio_neu.inverse()* cur_lio_neu;

       //initial guess
       SE3d init_pose = prev_kf->scd_opti_pose_ * relative_motion;
       init_pose.translation() = prev_kf->fst_opti_pose_.translation();

       SE3d res = init_pose;
       bool valid = true;
       bool ok = scd_ndt.Align(res);
       if (!ok) {
           std::cout<< "Inc_ndt failed, kf: "<< cur_kf->id_<<std::endl;
           valid = false;
       }
       NdtEval eval = scd_ndt.computeScore(res);

       //-----------check if the ndt res valid----------//

       if (eval.score_ > 5.0) {
          valid = false;
       }
       if (eval.valid_ratio_ < 0.80) {
           valid= false;
       }

       //-------------check if the change valid------------//
       const SE3d T_initial_result =
        init_pose.inverse() *
        res;

       const double translation_change =
           T_initial_result.translation().norm();

       const double rotation_change_rad =
           T_initial_result.so3().log().norm();

       const double rotation_change_deg =
           rotation_change_rad * 180.0 / M_PI;

       if (!std::isfinite(translation_change) ||
           !std::isfinite(rotation_change_deg)) {
           valid = false;
           }

       if (translation_change > 2.0) {
           valid = false;
       }

       if (rotation_change_deg > 5.0) {
           valid = false;
       }
       std::string valid_str =  valid? "valid": "invalid";
       std::cout<<"ndt: " << cur_kf->id_<< "vs " << prev_kf->id_<< std::endl;
       if(!valid) {
          std::cout<<"ndt res invalid!"<<std::endl;
          cur_kf->scd_opti_pose_  = init_pose;
       }
       else {
           cur_kf->scd_opti_pose_ = res;
       }
       transformCloud(cur_kf->cloud_ptr_, cur_kf->scd_opti_pose_);
       scd_ndt.AddCloud(cur_kf->cloud_ptr_);
       SE3d t_inv= cur_kf->scd_opti_pose_.inverse();
       transformCloud(cur_kf->cloud_ptr_, t_inv);
   }
}


bool Backend::computeSubmapRtk(const Submap& sm, Vec3d& gps_pos)const  {
    std::vector<Vec3d> pts;
    bool has_prev = false;
    Vec3d prev_pos = Vec3d::Zero();

    constexpr  double  kMaxGpsJump = 20.0;

    //debug span of submap
    auto fst_trans = sm.key_frames_.front()->fst_opti_pose_.translation();
    auto last_trans = sm.key_frames_.back()->fst_opti_pose_.translation();
    auto span =  (fst_trans - last_trans).norm();
    std::cout<< "[submap "<<sm.id_ << "]:" << span<<std::endl;

    for (const auto& kf  : sm.key_frames_) {
       if (!kf || !kf->rtk_valid_) {
           continue;
       }
       Vec3d cur_pos = kf->rtk_pose_.translation();
       if (has_prev) {
           double jump =
              (cur_pos - prev_pos).norm();

           if (jump > kMaxGpsJump) {
               std::cout << "[GPS Filter] reject KF "
                         << kf->id_
                         << " jump=" << jump
                         << " m"
                         << std::endl;
               prev_pos = cur_pos;
               continue;
           }
       }
        pts.push_back(cur_pos);
        prev_pos = cur_pos;
        has_prev = true;
    }
    if (pts.size() < 3) {
        return false;
    }

    Vec3d sum = Vec3d::Zero();
    for (const auto& p : pts) {
        sum += p;
    }

    gps_pos =
        sum / static_cast<double>(pts.size());

    return true;
}

void Backend::buildSubmapPointCloud() {
   //get all submaps
   auto& submaps = submap_manager_ptr_->getSubmaps();
   //for each submap, convert kf->cloud by its scd_opti_pose_, append to submap cloud
   for(auto& sm: submaps) {
       std::cout<< "build cloud for: "<<sm.id_ << std::endl;
       sm.point_cloud_ptr_ = std::make_shared<PointCloud>();
       sm.point_cloud_ptr_->clear();
       if (sm.key_frames_.empty()) {
           continue;
       }
       sm.T_w_s_init_ = sm.key_frames_.front()->scd_opti_pose_;

      for(auto& kf : sm.key_frames_ ) {
          if(!kf || !kf->cloud_ptr_) {
            continue;
          }
          SE3d T_s_k = sm.T_w_s_init_.inverse() * kf->scd_opti_pose_;
          //convert cloud by its' scd_opti_pose_
          PointCloud cloud_in_submap = transformCloudCopy(
                kf->cloud_ptr_,
                T_s_k);
          sm.point_cloud_ptr_->append(std::make_shared<PointCloud>(cloud_in_submap),
                cloud_in_submap.size());
      }
       if (sm.point_cloud_ptr_->hasBBox()) {
           std::cout << "bbox x: "
                     << sm.point_cloud_ptr_->minX()
                     << " ~ "
                     << sm.point_cloud_ptr_->maxX()
                     << std::endl;
       }
   }
}

std::shared_ptr<PointCloud>Backend::buildSlidingWindowCloud(size_t kf_id) {
    auto cloud = std::make_shared<PointCloud>();
    const auto center = key_frames_.find(kf_id);
    if (center == key_frames_.end() || !center->second) return cloud;
    const size_t half = static_cast<size_t>(std::max(0, backend_config_.loop_closure_options_.half_sw_size_));
    auto first = center;
    for(size_t n=0;n<half&&first!=key_frames_.begin();++n){auto previous=std::prev(first);if(center->second->time_-previous->second->time_>backend_config_.loop_closure_options_.window_half_sec_)break;first=previous;}
    auto last=std::next(center);
    for(size_t n=0;n<half&&last!=key_frames_.end();++n){if(last->second->time_-center->second->time_>backend_config_.loop_closure_options_.window_half_sec_)break;++last;}
    auto pose=[&](size_t id){return query_poses_.empty()?key_frames_.at(id)->scd_opti_pose_:query_poses_.at(id);};
    for(auto it=first;it!=last;++it){const auto& kf=it->second;if(!kf||!kf->cloud_ptr_)continue;
        const auto relative=pose(center->first).inverse()*pose(it->first);
        auto transformed=std::make_shared<PointCloud>(transformCloudCopy(kf->cloud_ptr_,relative));cloud->append(transformed,transformed->size());
    }
    return cloud;
}

std::pair<NdtEval,SE3d> Backend::runSubmapNdt(Submap& sm1, Submap& sm2, const SE3d& init_pose) {
    //Todo:might need to change to seperate config later
    IncNDT submap_ndt(backend_config_.inc_ndt_options_);
    submap_ndt.AddCloud(sm1.point_cloud_ptr_);
    SE3d pose= init_pose;
    // double score_before = submap_ndt.computeScore(init_pose);
    submap_ndt.SetSourceCloud(sm2.point_cloud_ptr_);
    bool ok  = submap_ndt.Align(pose);
    NdtEval eval = submap_ndt.computeScore(pose);
    if(!ok) {
        std::cout<< "Inc_ndt failed, sm: "<< sm2.id_<<std::endl;
    }
    std::cout<<"--------------------"<<std::endl;
    std::cout<<"ndt: " <<sm1.id_<<" vs "<<sm2.id_<<std::endl;
    std::cout<< "score: "<<eval.score_<<std::endl;
    std::cout<< "valid count: "<<eval.valid_count_<<std::endl;
    std::cout<< "valid ratio: "<<eval.valid_ratio_<<std::endl;
    return {eval, pose};
}

double Backend::bboxOverlapRatioXY(const Submap& a, const Submap& b)const {
    double ax0, ay0, ax1, ay1;
    double bx0, by0, bx1, by1;
    if (!a.getWorldBboxXY(ax0, ay0, ax1, ay1)) return 0.0;
    if (!b.getWorldBboxXY(bx0, by0, bx1, by1)) return 0.0;

    double ix = std::max(0.0, std::min(ax1, bx1) - std::max(ax0, bx0));
    double iy = std::max(0.0, std::min(ay1, by1) - std::max(ay0, by0));

    double inter = ix * iy;

    double area_a = std::max(1e-6, (ax1 - ax0) * (ay1 - ay0));
    double area_b = std::max(1e-6, (bx1 - bx0) * (by1 - by0));

    double uni = area_a + area_b - inter;
    return inter / std::max(1e-6, uni);
}


void Backend::runLoopClosure(){
   buildSubmapPointCloud();
   // run ndt between adj submaps, so submap has submap_pose, and add graph node
   auto& submaps =  submap_manager_ptr_->getSubmaps();
   if (submaps.size() <2) {
     std::cout<<"Not enogh submap pairs"<<std::endl;
   }

   PoseGraphOptimizer optimizer(OptimizationStage::SUBMAP_NDT);
   std::vector<PoseGraphOptimizer::Node>node_vect;
   std::vector<PoseGraphOptimizer::Edge>edge_vect;
   //
   for(int i=0 ;i+1<submaps.size(); i++) {
       Submap& sm = submaps[i];
       PoseGraphOptimizer::Node node;
       node.id_ = sm.id_;
       node.pose_init_ = sm.T_w_s_init_;
       // No GPS in this stage
       node.has_gps_ = false;
       node_vect.push_back(node);
    }

    //submap lv registration
    std::vector<LoopConstraint> loop_constraints;
    //runSubmapRegistration(loop_constraints);

    //find loop pairs
    std::vector<LoopPair> loop_vect;
    std::vector<LoopPair>sparse_vect;
    findLoopPairsViaKfs(loop_vect);

    selectSparseLoopPairs(loop_vect, sparse_vect);
    markLoopClosureKfs(sparse_vect);

    //loop registration and add constrain
    //runLoopRegistration(sparse_vect, loop_constraints);

    //convert loop constrains to edges
    for (const auto& con : loop_constraints) {
       if (!con.valid_) {
          continue;
       }
      PoseGraphOptimizer::Edge edge;
      edge.id_i_ = con.idx_i_;
      edge.id_j_ = con.idx_j_;
      edge.T_i_j_ = con.T_i_j_;
      edge.info_ = con.info_;
      edge_vect.push_back(edge);
    }

   // optimizer.setNodes(node_vect);
   // optimizer.setEdges(edge_vect);
   // bool ok = optimizer.optimize(30);
   //  if (!ok) {
   //      std::cout << "Submap graph optimization failed" << std::endl;
   //      return;
   //  }
   //  std::map<size_t, SE3d> optimized_poses;
   //  optimizer.getOptimizedPoses(optimized_poses);
   //  for (auto& sm : submaps) {
   //      auto iter = optimized_poses.find(sm.id_);
   //
   //      if (iter == optimized_poses.end()) {
   //          continue;
   //      }
   //
   //      sm.T_w_s_opti_ = optimized_poses[sm.id_];
   //      std::cout<<"update submap: "<< sm.id_<< " pose"<<std::endl;
   //  }

   applySubmapCorrectionToKeyframes();
}
void Backend::findLoopPairsForKf(size_t kf_id, std::vector<LoopPair>& loop_pairs) {
   loop_pairs.clear();
   LoopClosureOptions options = backend_config_.loop_closure_options_;
    int  diff = kf_id- options.loop_min_kf_gap_;
    if (diff < 0) {
       return;
    }
   for(int i=0; i<= static_cast<int>(kf_id)-options.loop_min_kf_gap_; i++ ) {
       if (key_frames_.find(i) == key_frames_.end()) {
          continue;
       }
       auto& kf_i  = key_frames_[i];
       auto& kf_j  = key_frames_[kf_id];
       if (!kf_i || !kf_j) {
          continue;
       }
       Vec3d pi = kf_i->scd_opti_pose_.translation();
       Vec3d pj = kf_j->scd_opti_pose_.translation();

       double dist_xy = (pi.head<2>() - pj.head<2>()).norm();
       double dist_z = std::abs(pi.z() - pj.z());

       if (dist_xy > options.xy_thresh_ || dist_z > options.z_thresh_) {
           continue;
       }
       if(kf_i->submap_id_ == INVALID_ID || kf_j->submap_id_ == INVALID_ID ) {
           continue;
       }
       std::vector<LoopPair> loop_pairs_tmp = makeSubmapPairsFromKfPair(kf_i, kf_j);
       for (const auto& lp : loop_pairs_tmp) {
           if (lp.overlap_ratio_ >options.min_overlap_) {
               loop_pairs.push_back(lp);
           }
           std::cout << "[Raw KF -> Submap Candidate] "
                     << "kf " << lp.kf_idx_i_ << " vs " << lp.kf_idx_j_
                     << " => submap " << lp.idx_i_ << " vs " << lp.idx_j_
                     << " overlap = " << lp.overlap_ratio_
                     << " dist = " << lp.dist_xy_
                     << std::endl;
       }
   }
}

LoopPair Backend::chooseBestLoopPair(std::vector<LoopPair>& loop_pairs) {
    if (loop_pairs.empty()) {
        return LoopPair();   // make sure LoopPair default idx_i_/idx_j_ = INVALID or -1
    }

    double best_score = std::numeric_limits<double>::infinity();
    LoopPair best_pair;
    bool found = false;

    LoopClosureOptions options = backend_config_.loop_closure_options_;
    auto& submaps = submap_manager_ptr_->getSubmaps();

    for (auto& pair : loop_pairs) {
        if (pair.idx_i_ < 0 || pair.idx_j_ < 0) {
            continue;
        }

        if (pair.idx_i_ >= static_cast<int>(submaps.size()) ||
            pair.idx_j_ >= static_cast<int>(submaps.size())) {
            continue;
            }

        auto& sm_i = submaps[pair.idx_i_];
        auto& sm_j = submaps[pair.idx_j_];

        SE3d init =
            sm_i.T_w_s_init_.inverse() * sm_j.T_w_s_init_;

        auto res = runSubmapNdt(sm_i, sm_j, init);

        NdtEval eval = res.first;
        SE3d delta = res.second;

        pair.T_i_j_init_ = delta;
        pair.ndt_score_ = eval.score_;

        if (!std::isfinite(eval.score_)) {
            continue;
        }

        if (eval.score_ > options.score_gate_) {
            continue;
        }

        if (eval.valid_count_ < options.valid_count_gate_) {
            continue;
        }

        if (eval.valid_ratio_ < options.valid_ratio_gate_) {
            continue;
        }

        if (eval.score_ < best_score) {
            best_score = eval.score_;
            best_pair = pair;
            found = true;
        }
    }

    std::cout << "There are: " << loop_pairs.size() << " pairs" << std::endl;

    if (!found) {
        std::cout << "No valid loop pair after NDT gate." << std::endl;
        return LoopPair();
    }

    std::cout << "best: "
              << best_pair.idx_i_ << " vs "
              << best_pair.idx_j_ << std::endl;

    std::cout << "best_score: "
              << best_pair.ndt_score_ << std::endl;

    return best_pair;
}

void Backend::removeDuplicateSubmapPairs(std::vector<LoopPair>& pairs)
{
    std::set<std::pair<int, int>> visited;
    std::vector<LoopPair> unique;
    unique.reserve(pairs.size());
    for (const auto& p : pairs) {
        int a = std::min(p.idx_i_, p.idx_j_);
        int b = std::max(p.idx_i_, p.idx_j_);

        if (a < 0 || b < 0) {
            continue;
        }

        std::pair<int, int> key(a, b);

        if (visited.count(key)) {
            continue;
        }

        visited.insert(key);
        unique.push_back(p);
    }

    pairs.swap(unique);
}

void Backend::updateOptimizationToSubmap(const std::map<size_t, SE3d>&optimized_map, std::vector<Submap>& submap_vect) {
    // update the visited submap
    int last_id = -1;
    for (auto& it : optimized_map) {
       size_t submap_id = it.first;
       if (submap_id != (last_id +1)) {
          std::cout<<"When Update Submap, the submap order is corrupted"<< std::endl;
       }
       SE3d opt_pose = it.second;
       auto& submap = submap_vect[submap_id];
       submap.T_w_s_opti_ = opt_pose;
       last_id = submap_id;
   }
   //update the following submap, which means these submao has not been optimized yet
   for (size_t id = last_id+1; id < submap_vect.size(); id++) {
       Submap& sm =  submap_vect[id];
       Submap& last_sm = submap_vect[id-1];
       SE3d delta = last_sm.T_w_s_init_.inverse() * sm.T_w_s_init_;
       sm.T_w_s_opti_ = last_sm.T_w_s_opti_ * delta;
   }
}


void Backend::runLoopClosureLocalToGlobal(loop_callback callback) {
    // kf->scd_opti_pose_ -> submap.T_w_s_init
    clearKfsLoop();
    buildSubmapPointCloud();
    auto& submaps =  submap_manager_ptr_->getSubmaps();
    if (submaps.size() <2) {
        std::cout<<"Not enogh submap pairs"<<std::endl;
    }
    PoseGraphOptimizer optimizer(OptimizationStage::SUBMAP_NDT);
    std::vector<PoseGraphOptimizer::Node>node_vect;
    std::vector<PoseGraphOptimizer::Edge>edge_vect;
    std::unordered_set<size_t>visited_submap;
    Mat6d loop_info  = Mat6d::Zero();
    loop_info(0,0) = 50.0;
    loop_info(1,1) = 50.0;
    loop_info(2,2) = 30.0;
    loop_info(3,3) = 20.0;
    loop_info(4,4) = 20.0;
    loop_info(5,5) = 50.0;
    int cnt = 0;
    size_t last_kf_id = INVALID_ID;
    for(auto it : key_frames_) {
       cnt++;
       auto& kf = it.second;
       if (!kf) {
         continue;
       }
       bool early_exit = false;
       std::vector<LoopPair> loop_pairs;
       //find the loop pair of kf
       findLoopPairsForKf( kf->id_, loop_pairs);
       removeDuplicateSubmapPairs(loop_pairs);
       //
       if (loop_pairs.size() ==0) {
            early_exit = true;
       }
        std::cout<< "find " << loop_pairs.size() << "for" << kf->id_<<std::endl;
       //find the loop pair with the best score(best overlap + best ndt)
       LoopPair best_pair = chooseBestLoopPair(loop_pairs);
       if (best_pair.idx_i_ == INVALID_ID || best_pair.idx_j_ == INVALID_ID) {
         early_exit = true;
       }

       if(early_exit) {
           if (kf->submap_id_ != INVALID_ID) {
               if (!visited_submap.count(kf->submap_id_)) {
                   Submap& sm = submaps[kf->submap_id_];
                   PoseGraphOptimizer::Node node;
                   node.id_ = sm.id_;
                   node.pose_init_ = sm.T_w_s_init_;
                   // No GPS in this stage
                   node.has_gps_ = false;
                   node_vect.push_back(node);
               }
               visited_submap.insert(kf->submap_id_);
           }
           continue;
       }
       //make sure the current submap has been added
       if(!visited_submap.count(kf->submap_id_)) {
           Submap& sm = submaps[kf->submap_id_];
           PoseGraphOptimizer::Node node;
           node.id_ = sm.id_;
           node.pose_init_ = sm.T_w_s_init_;
           // No GPS in this stage
           node.has_gps_ = false;
           node_vect.push_back(node);
           visited_submap.insert(kf->submap_id_);
       }
        last_kf_id = kf->id_;
        //add loop edge
        std::cout<< "---------------------------"<<std::endl;
        std::cout<< "pair: " << best_pair.idx_i_ << " vs " << best_pair.idx_j_ << std::endl;

        edge_vect.clear();
        PoseGraphOptimizer::Edge loop_edge;
        loop_edge.id_i_ = best_pair.idx_i_;
        loop_edge.id_j_ = best_pair.idx_j_;
        loop_edge.T_i_j_ = best_pair.T_i_j_init_;
        loop_edge.info_ = loop_info;
        edge_vect.push_back(loop_edge);

        //mark the kf
       markLoopClosureKfs({best_pair});

        //each time, the already added edge should update
       addCurrentNdtEdges(node_vect, edge_vect);

       //optimize current edge and node
       optimizer.setEdges(edge_vect);
       optimizer.setNodes(node_vect);
        bool ok = optimizer.optimize(30);
        if (!ok) {
            std::cout << "Submap graph optimization failed" << std::endl;
            return;
        }
        std::map<size_t, SE3d> optimized_poses_local;
        optimizer.getOptimizedPoses(optimized_poses_local);
        //update the submap and kfs, T_w_s_opti_
        updateOptimizationToSubmap(optimized_poses_local, submaps);
        applySubmapCorrectionToKeyframes();
        callback(cnt);
        //send signal to visnode, wait until the signal back
    }

    //After all local optimization down. run global optimization
    bool ok = optimizer.optimize(30);
    if (!ok) {
        std::cout << "Global optimization failed" << std::endl;
        return;
    }
    std::map<size_t, SE3d> optimized_poses_global;
    optimizer.getOptimizedPoses(optimized_poses_global);
    updateOptimizationToSubmap(optimized_poses_global,submaps);
    applySubmapCorrectionToKeyframes();
    return;
}

void Backend::applySubmapCorrectionToKeyframes() {
   auto& submaps = submap_manager_ptr_->getSubmaps();
   for (auto& sm : submaps) {
       SE3d delta =
             sm.T_w_s_opti_ * sm.T_w_s_init_.inverse();
       double trans =delta.translation().norm();
       if (trans >0.5) {
           std::cout << "id: " << sm.id_ << std::endl;
           std::cout<< "trans: "<<trans<<std::endl;
       }
       for (auto& kf : sm.owned_keyframes_) {
           kf->loop_opti_pose_ =
               delta * kf->scd_opti_pose_;
       }
   }
}

std::pair<SE3d, NdtEval>Backend::runSlidingWindowNdt(size_t kf_i, size_t kf_j) {
    //make sure sanity check is done outside, so kf_i and kf_j has sliding window
    std::shared_ptr<PointCloud> sw_l = buildSlidingWindowCloud(kf_i);
    std::shared_ptr<PointCloud> sw_r = buildSlidingWindowCloud(kf_j);
    if (sw_l->empty() || sw_r->empty()) {
        return {SE3d(), {std::numeric_limits<double>::infinity(), 0, 0.0}};
    }


    auto pose=[&](size_t id){return query_poses_.empty()?key_frames_.at(id)->scd_opti_pose_:query_poses_.at(id);};
    SE3d init_pose=pose(kf_i).inverse()*pose(kf_j);NdtEval eval;
    for(double size:backend_config_.loop_closure_options_.voxel_sizes_) {
        auto options=backend_config_.inc_ndt_options_;options.voxel_size_=size;
        IncNDT ndt(options);ndt.setStopRequested(stop_requested_);ndt.AddCloud(sw_l);ndt.SetSourceCloud(sw_r);
        if(!ndt.Align(init_pose)||!init_pose.matrix().allFinite()||stopped())return {init_pose,NdtEval{}};
        eval=ndt.computeScore(init_pose);
    }
    return {init_pose, eval};
}

void Backend::markLoopSlidingWindow(size_t kf_i, size_t kf_j) {
    auto options = backend_config_.loop_closure_options_;

    int half = static_cast<int>(options.half_sw_size_);
    auto mark = [&](size_t center_id, int role) {
        auto center = key_frames_.find(center_id);
        if (center == key_frames_.end()) return;
        auto first=center;
        for(int n=0;n<half&&first!=key_frames_.begin();++n){auto previous=std::prev(first);if(center->second->time_-previous->second->time_>options.window_half_sec_)break;first=previous;}
        auto last=std::next(center);
        for(int n=0;n<half&&last!=key_frames_.end();++n){if(last->second->time_-center->second->time_>options.window_half_sec_)break;++last;}
        for (auto it = first; it != last; ++it) {
            if (!it->second) continue;
            it->second->is_loop_closure_ = true;
            it->second->loop_role_ = role;
        }
    };

    mark(kf_i, 1);
    mark(kf_j, 2);
}

LoopPair Backend::findLoopPairsSlidingWindow(size_t kf_id) {
    const auto& o=backend_config_.loop_closure_options_;LoopPair best;
    auto query=key_frames_.find(kf_id);if(query==key_frames_.end())return best;
    std::vector<std::pair<double,size_t>> candidates;
    for(auto it=key_frames_.begin();it!=query;++it) {
        const auto id=it->first;if(query->second->time_-it->second->time_<o.min_time_sec_ || query_travel_.at(kf_id)-query_travel_.at(id)<o.min_travel_m_)continue;
        const auto d=query_poses_.at(kf_id).translation()-query_poses_.at(id).translation();
        if(d.head<2>().norm()>o.kf_xy_thresh_||std::abs(d.z())>o.kf_z_thresh_)continue;
        bool duplicate=false;for(const auto& loop:evaluation_loops_)if(std::abs(query->second->time_-key_frames_.at(loop.kf_idx_j_)->time_)<o.duplicate_sec_&&std::abs(it->second->time_-key_frames_.at(loop.kf_idx_i_)->time_)<o.duplicate_sec_)duplicate=true;
        if(!duplicate)candidates.emplace_back(d.head<2>().norm(),id);
    }
    std::sort(candidates.begin(),candidates.end());std::vector<double> historical_times;size_t tested=0;
    for(const auto& candidate:candidates) {
        if(stopped())break;const size_t i=candidate.second;const double time=key_frames_.at(i)->time_;
        bool same_group=false;for(double previous:historical_times)if(std::abs(previous-time)<o.history_group_sec_)same_group=true;
        if(same_group)continue;if(tested++>=o.max_candidates_)break;historical_times.push_back(time);
        const auto initial=query_poses_.at(i).inverse()*query_poses_.at(kf_id);const auto result=runSlidingWindowNdt(i,kf_id);
        SE3d measurement=result.first;WallAwareLoopResult wall;double translation=INFINITY,angle=INFINITY;
        std::string reason;LoopPointSupport forward,reverse;
        std::shared_ptr<PointCloud> target,source;
        if(!measurement.matrix().allFinite()||!std::isfinite(result.second.score_))reason="registration_failed";
        else {
            target=buildSlidingWindowCloud(i);source=buildSlidingWindowCloud(kf_id);
            if(o.wall_aware_.enabled) {
                wall=refineWallPointToPlane(*target,*source,measurement,o.wall_aware_,stop_requested_);
                if(!wall.success)reason=wall.reason;
                else measurement=wall.pose;
            }
        }
        if(reason.empty()) {
            const auto correction=initial.inverse()*measurement;
            translation=correction.translation().norm();angle=correction.so3().log().norm()*180/M_PI;
            if(translation>o.max_correction_m_||angle>o.max_correction_deg_)reason="correction_too_large";
        }
        if(reason.empty()) {
            forward=loopPointSupport(*target,*source,measurement,o.point_distance_m_);reverse=loopPointSupport(*source,*target,measurement.inverse(),o.point_distance_m_);
            if(forward.ratio<o.min_bidirectional_overlap_||reverse.ratio<o.min_bidirectional_overlap_)reason="insufficient_overlap";
            else if(forward.matched<o.min_matched_points_||reverse.matched<o.min_matched_points_)reason="insufficient_matches";
            else if(forward.rmse>o.max_point_rmse_m_||reverse.rmse>o.max_point_rmse_m_)reason="point_rmse_too_large";
            else reason="verified";
        }
        if(reason=="verified" && o.consecutive_verification_)reason=verifyConsecutiveLoop(i,kf_id,measurement);
        if(loop_candidate_log_) {loop_candidate_log_<<i<<','<<kf_id<<','<<query->second->time_-time<<','<<candidate.first<<','<<result.second.score_<<','<<translation<<','<<angle<<','<<forward.ratio<<','<<reverse.ratio<<','<<forward.rmse<<','<<reverse.rmse<<','<<wall.correspondences<<','<<wall.rmse_m<<','<<wall.observable_modes<<','<<wall.weakest_observable_ratio<<','<<wall.floor_correspondences<<','<<wall.floor_rmse_m<<','<<reason<<'\n';loop_candidate_log_.flush();}
        if(reason!="verified")continue;
        const double quality=std::min(forward.ratio,reverse.ratio);
        if(best.kf_idx_i_==INVALID_ID||quality>best.overlap_ratio_) {
            best.kf_idx_i_=i;best.kf_idx_j_=kf_id;best.T_i_j_init_=measurement;best.ndt_score_=result.second.score_;best.dist_xy_=candidate.first;best.overlap_ratio_=quality;
            if(o.wall_aware_.enabled)best.information_shape_=wall.information_shape;
            best.wall_correspondences_=wall.correspondences;best.wall_observable_modes_=wall.observable_modes;best.wall_rmse_m_=wall.rmse_m;
            best.floor_correspondences_=wall.floor_correspondences;best.floor_rmse_m_=wall.floor_rmse_m;
        }
    }
    return best;
}

// Consecutive scans verify one window-to-window constraint; they do not add graph edges.
std::string Backend::verifyConsecutiveLoop(size_t anchor,size_t query,const SE3d& measurement) {
    const auto& o=backend_config_.loop_closure_options_;
    auto current=key_frames_.find(query);
    std::vector<size_t> support;
    for(size_t n=0;n<o.consecutive_frames_;++n) {
        if(current==key_frames_.end())return "confirmation_missing_frames";
        support.push_back(current->first);
        if(n+1<o.consecutive_frames_) {
            if(current==key_frames_.begin())return "confirmation_missing_frames";
            auto previous=std::prev(current);
            if(current->second->time_-previous->second->time_>o.consecutive_max_gap_sec_)return "confirmation_time_gap";
            current=previous;
        }
    }
    auto target=buildSlidingWindowCloud(anchor);
    for(size_t id:support) {
        if(stopped())return "confirmation_stopped";
        auto source=key_frames_.at(id)->cloud_ptr_;
        SE3d aligned=query_poses_.at(anchor).inverse()*query_poses_.at(id);
        bool ok=source && !source->empty();
        for(double voxel:o.voxel_sizes_) {
            if(!ok)break;
            auto options=backend_config_.inc_ndt_options_;options.voxel_size_=voxel;
            IncNDT ndt(options);ndt.setStopRequested(stop_requested_);ndt.AddCloud(target);ndt.SetSourceCloud(source);
            ok=ndt.Align(aligned) && aligned.matrix().allFinite();
        }
        if(ok && o.wall_aware_.enabled) {
            const auto wall=refineWallPointToPlane(*target,*source,aligned,o.wall_aware_,stop_requested_);
            ok=wall.success;
            if(ok)aligned=wall.pose;
        }
        LoopPointSupport point_support;
        double translation=INFINITY,angle=INFINITY;std::string result="registration_failed";
        if(ok) {
            point_support=loopPointSupport(*target,*source,aligned,o.point_distance_m_);
            const auto disagreement=loopConsensusDisagreement(measurement,aligned,query_poses_.at(id),query_poses_.at(query));
            translation=disagreement.translation_m;angle=disagreement.rotation_deg;
            if(point_support.ratio<o.min_bidirectional_overlap_ || point_support.rmse>o.max_point_rmse_m_ || point_support.matched<o.consecutive_min_matches_)result="insufficient_support";
            else if(translation>o.consensus_translation_m_ || angle>o.consensus_rotation_deg_)result="inconsistent_correction";
            else result="confirmed";
        }
        if(loop_consensus_log_) {
            loop_consensus_log_<<anchor<<','<<query<<','<<id<<','<<point_support.ratio<<','<<point_support.rmse<<','<<point_support.matched<<','<<translation<<','<<angle<<','<<result<<'\n';loop_consensus_log_.flush();
        }
        if(result!="confirmed")return "confirmation_"+result;
    }
    return "verified";
}

double Backend::getYaw(const SE3d& pose) {
    const Mat3d& R = pose.so3().matrix();
    return std::atan2(R(1, 0), R(0, 0));
}

Mat3d Backend::makeYawRotation(double yaw) {
   return Eigen::AngleAxisd(yaw, Vec3d::UnitZ()).toRotationMatrix();
}
SE3d Backend::keepXYAndYawOnly(const SE3d& optimized_pose,
                 const SE3d& reference_pose) {
    const double optimized_yaw = getYaw(optimized_pose);
    const double reference_yaw = getYaw(reference_pose);

    const Eigen::Matrix3d R_reference =
       reference_pose.so3().matrix();

    // Remove the original yaw, leaving the original roll/pitch component.
    const Eigen::Matrix3d R_tilt =
        makeYawRotation(-reference_yaw) * R_reference;

    // Apply optimized yaw while preserving original tilt.
    const Eigen::Matrix3d R_new =
        makeYawRotation(optimized_yaw) * R_tilt;

    Eigen::Vector3d t_new = reference_pose.translation();
    t_new.x() = optimized_pose.translation().x();
    t_new.y() = optimized_pose.translation().y();

    // t_new.z() remains from reference_pose.
    return SE3d(R_new, t_new);
}

SE3d Backend::keepXYZAndYawOnly(const SE3d& optimized_pose,
                 const SE3d& reference_pose) {
    const SE3d planar = keepXYAndYawOnly(optimized_pose, reference_pose);
    Vec3d translation = planar.translation();
    translation.z() = optimized_pose.translation().z();
    return SE3d(planar.so3(), translation);
}

void Backend::updateLoopToKf(const std::map<size_t, SE3d>& optimized_map) {
    auto last = key_frames_.end();
    for (const auto& pose : optimized_map) {
        auto kf = key_frames_.find(pose.first);
        if (kf == key_frames_.end() || !kf->second || !pose.second.matrix().allFinite()) continue;
        const auto& options=backend_config_.loop_closure_options_;
        if(options.full_3d_)kf->second->loop_opti_pose_=pose.second;
        else if(options.wall_aware_.enabled && options.wall_aware_.optimize_z)
            kf->second->loop_opti_pose_=keepXYZAndYawOnly(pose.second,kf->second->scd_opti_pose_);
        else kf->second->loop_opti_pose_=keepXYAndYawOnly(pose.second,kf->second->scd_opti_pose_);
        last = kf;
    }
    if (last == key_frames_.end()) return;
    for (auto it = std::next(last); it != key_frames_.end(); ++it) {
        if (!it->second) continue;
        const SE3d delta = last->second->scd_opti_pose_.inverse() * it->second->scd_opti_pose_;
        it->second->loop_opti_pose_ = last->second->loop_opti_pose_ * delta;
        last = it;
    }
}

void Backend::runKfLoopClosureLocalToGlobal(loop_callback callback) {
    if (stopped()) return;
    const bool integrated=backend_config_.use_gnss_ && backend_config_.gnss_xy_only_ && backend_config_.gnss_local_to_global_;
    if(backend_config_.use_gnss_ && backend_config_.gnss_xy_only_ && !integrated) { runFixedLoopGnssXY(callback); return; }
    evaluation_loops_.clear();
    if (key_frames_.empty()) {
        return;
    }
    LoopClosureOptions options = backend_config_.loop_closure_options_;
    clearKfsLoop();
    PoseGraphOptimizer optimizer(integrated?OptimizationStage::GNSS_XY_PLANAR:OptimizationStage::SUBMAP_NDT);
    std::vector<PoseGraphOptimizer::Node>node_vect;
    std::vector<PoseGraphOptimizer::Edge>edge_vect;

    Mat6d odom_info = Mat6d::Zero();
    odom_info(0,0) = 1000.0;
    odom_info(1,1) = 1000.0;
    odom_info(2,2) = 1000.0;

    odom_info(3,3) = 1000.0;
    odom_info(4,4) = 1000.0;
    odom_info(5,5) = 1000.0;

    Mat6d loop_info  = Mat6d::Zero();
    loop_info(0,0) = 10.0;
    loop_info(1,1) = 10.0;
    loop_info(2,2) = 10.0;
    loop_info(3,3) = 10.0;
    loop_info(4,4) = 10.0;
    loop_info(5,5) = 10.0;

    if(integrated) {
        odom_info=backend_config_.lio_information_*Mat6d::Identity();
        loop_info=backend_config_.loop_information_*Mat6d::Identity();
    }
    Mat3d  gps_info = Mat3d::Zero();
    gps_info(0,0) = 0.0;
    gps_info(1,1) = 0.0;
    gps_info(2,2) = 0.0;
    optimizer.setStopRequested(stop_requested_);
    if(integrated)gps_info(0,0)=gps_info(1,1)=backend_config_.gnss_information_xy_;
    optimizer.setGpsInfo(gps_info);
    optimizer.setGpsHuberDelta(backend_config_.gnss_huber_delta_);
    std::map<size_t,SE3d> initial;
    std::set<size_t> selected_gnss;
    double last_gnss=-INFINITY;
    for(const auto& entry:key_frames_)initial[entry.first]=entry.second->loop_opti_pose_;
    std::ofstream stage_log;
    if(integrated && !backend_config_.gnss_diagnostics_path_.empty()) {
        stage_log.open(backend_config_.gnss_diagnostics_path_+".stages.csv");
        if(!stage_log)throw std::runtime_error("Cannot open joint stage diagnostics");
        stage_log<<"stage,nodes,odom_edges,loop_edges,gnss_factors\n";
    }
    auto record_stage=[&](const char* stage) {
        if(!integrated)return;
        std::cout<<"[Joint local-to-global] "<<stage<<": nodes="<<node_vect.size()<<", loops="<<evaluation_loops_.size()<<", XY GNSS="<<selected_gnss.size()<<"\n";
        if(stage_log){stage_log<<stage<<','<<node_vect.size()<<','<<(node_vect.empty()?0:node_vect.size()-1)<<','<<evaluation_loops_.size()<<','<<selected_gnss.size()<<'\n';stage_log.flush();}
    };

    double last_query_time=-INFINITY;
    query_poses_.clear();query_travel_.clear();double travel=0;SE3d previous_query;bool first_query=true;
    for(const auto& entry:key_frames_) {
        query_poses_[entry.first]=entry.second->lidar_pose_;
        if(!first_query)travel+=(entry.second->lidar_pose_.translation()-previous_query.translation()).norm();
        query_travel_[entry.first]=travel;previous_query=entry.second->lidar_pose_;first_query=false;
    }
    if(!backend_config_.loop_candidate_log_path_.empty()) {
        loop_candidate_log_.open(backend_config_.loop_candidate_log_path_,std::ios::trunc);
        if(!loop_candidate_log_)throw std::runtime_error("Cannot open loop candidate log");
        loop_candidate_log_<<"kf_i,kf_j,time_separation_s,query_xy_m,ndt_score,correction_m,correction_deg,forward_overlap,reverse_overlap,forward_rmse_m,reverse_rmse_m,wall_correspondences,wall_rmse_m,wall_observable_modes,wall_weakest_eigen_ratio,floor_correspondences,floor_rmse_m,result\n";
        loop_candidate_log_<<std::setprecision(17);
        if(options.consecutive_verification_) {
            loop_consensus_log_.open(backend_config_.loop_candidate_log_path_+".consensus.csv",std::ios::trunc);
            if(!loop_consensus_log_)throw std::runtime_error("Cannot open loop consensus diagnostics");
            loop_consensus_log_<<"anchor,query,support_frame,overlap,rmse_m,matched,translation_disagreement_m,rotation_disagreement_deg,result\n"<<std::setprecision(17);
        }
    }

    int cnt = 0;
    std::shared_ptr<KeyFrame> previous;

    for (auto& it : key_frames_) {
        if (it.second && !integrated) {
            it.second->loop_opti_pose_ = it.second->scd_opti_pose_;
        }
    }

    for(auto it : key_frames_) {
        if (stopped()) return;

        auto& kf = it.second;
        if (!kf || !kf->scd_opti_pose_.matrix().allFinite()) {
            continue;
        }
        ++cnt;
        std::cout<<"process:" << kf->id_<<std::endl;
        //Add optimization node
        PoseGraphOptimizer::Node node;
        node.id_ = kf->id_;
        node.pose_init_ = kf->loop_opti_pose_;
        node.t_ = kf->time_;
        node.has_gps_ = integrated && kf->rtk_valid_ && kf->rtk_pose_.translation().head<2>().allFinite() && kf->time_-last_gnss>=backend_config_.gnss_min_interval_sec_;
        if(node.has_gps_) {
            node.gps_pos_=Vec3d(kf->rtk_pose_.translation().x(),kf->rtk_pose_.translation().y(),0);
            last_gnss=kf->time_;selected_gnss.insert(kf->id_);
        }
        node_vect.push_back(node);
        if (previous) {
            PoseGraphOptimizer::Edge edge;
            edge.id_i_ = previous->id_;
            edge.id_j_ = kf->id_;
            // Measurements stay in the original corrected-odometry trajectory.
            edge.T_i_j_ = previous->scd_opti_pose_.inverse() * kf->scd_opti_pose_;
            edge.info_ = odom_info;
            edge_vect.push_back(edge);
        }
        previous = kf;

        //find loop pair
        if(kf->time_-last_query_time<options.query_interval_sec_)continue;
        last_query_time=kf->time_;
        LoopPair best_pair = findLoopPairsSlidingWindow(kf->id_);
        if (best_pair.kf_idx_i_ == INVALID_ID || best_pair.kf_idx_j_ == INVALID_ID|| best_pair.ndt_score_ == std::numeric_limits<double>::infinity()) {
          continue;
        }


        PoseGraphOptimizer::Edge loop_edge;
        loop_edge.id_i_ = best_pair.kf_idx_i_;
        loop_edge.id_j_ = best_pair.kf_idx_j_;
        loop_edge.T_i_j_ = best_pair.T_i_j_init_;
        loop_edge.info_ = options.wall_aware_.enabled
            ? loop_info(0,0) * best_pair.information_shape_
            : loop_info;
        edge_vect.push_back(loop_edge);
        evaluation_loops_.push_back(best_pair);

        // Keep every accepted loop window marked so the final replay shows the
        // complete set of loop closures, rather than only the latest pair.
        markLoopSlidingWindow(best_pair.kf_idx_i_, best_pair.kf_idx_j_);

        if (stopped()) return;
        //optimize current edge and node
        optimizer.setEdges(edge_vect);
        optimizer.setNodes(node_vect);
        record_stage("local");
        bool ok = optimizer.optimize(30);
        if (!ok) {
            throw std::runtime_error("Local graph optimization failed");
        }
        std::map<size_t, SE3d> optimized_poses_local;
        optimizer.getOptimizedPoses(optimized_poses_local);

        if(integrated) {
            for(const auto& pose:optimized_poses_local)key_frames_.at(pose.first)->loop_opti_pose_=pose.second;
            // Continue the unprocessed baseline with the last optimized XY/yaw correction.
            // Inspection includes future window frames, which must share this correction.
            if(!optimized_poses_local.empty()) {
                const auto& last=*optimized_poses_local.rbegin();
                const Mat3d delta=last.second.rotationMatrix()*initial.at(last.first).rotationMatrix().transpose();
                const SO3d rotation=SO3d::exp(Vec3d(0,0,std::atan2(delta(1,0),delta(0,0))));
                Vec3d translation=last.second.translation()-rotation*initial.at(last.first).translation();translation.z()=0;
                for(auto next=key_frames_.upper_bound(last.first);next!=key_frames_.end();++next) {
                    const auto& reference=initial.at(next->first);
                    next->second->loop_opti_pose_=SE3d(rotation*reference.so3(),rotation*reference.translation()+translation);
                }
            }
        } else updateLoopToKf(optimized_poses_local);
        for (auto& node : node_vect) node.pose_init_ = key_frames_.at(node.id_)->loop_opti_pose_;
        if (callback) callback(static_cast<int>(std::min(key_frames_.size(),static_cast<size_t>(cnt+options.half_sw_size_))));
    }
    if (stopped()) return;
    //After all local optimization down. run global optimization
    optimizer.setNodes(node_vect);
    optimizer.setEdges(edge_vect);
    record_stage("global");
    bool ok = optimizer.optimize(30);
    if (!ok) {
        throw std::runtime_error("Global optimization failed");
    }
    std::map<size_t, SE3d> optimized_poses_global;
    optimizer.getOptimizedPoses(optimized_poses_global);
    if(integrated) {
        for(const auto& pose:optimized_poses_global)key_frames_.at(pose.first)->loop_opti_pose_=pose.second;
        if(!backend_config_.gnss_diagnostics_path_.empty()) {
            std::ofstream log(backend_config_.gnss_diagnostics_path_);
            if(!log)throw std::runtime_error("Cannot open joint GNSS diagnostics");
            log<<"kf_id,time,gnss_valid,selected,initial_xy_residual_m,final_xy_residual_m,pose_change_m,z_change_m\n"<<std::setprecision(17);
            for(const auto& entry:key_frames_) {
                const auto& kf=entry.second;const auto a=initial.at(entry.first).translation(),b=kf->loop_opti_pose_.translation();
                log<<entry.first<<','<<kf->time_<<','<<kf->rtk_valid_<<','<<selected_gnss.count(entry.first)<<',';
                if(kf->rtk_valid_)log<<(a-kf->rtk_pose_.translation()).head<2>().norm()<<','<<(b-kf->rtk_pose_.translation()).head<2>().norm();else log<<"nan,nan";
                log<<','<<(b-a).norm()<<','<<b.z()-a.z()<<'\n';
            }
            log.close();if(!log)throw std::runtime_error("Cannot finish joint diagnostics");
        }
    } else updateLoopToKf(optimized_poses_global);
    if (callback) callback(node_vect.size());
}

void Backend::writeEvaluationLoops(const std::string& path) const {
    const auto temporary = path + ".tmp";
    std::ofstream out(temporary);
    out << "# lio_evaluation_loops_v1: id_i id_j tx ty tz qx qy qz qw ndt_score\n";
    out << std::setprecision(18);
    for (const auto& loop : evaluation_loops_) {
        const auto t = loop.T_i_j_init_.translation();
        const auto q = loop.T_i_j_init_.unit_quaternion();
        out << loop.kf_idx_i_ << ' ' << loop.kf_idx_j_ << ' '
            << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
            << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
            << ' ' << loop.ndt_score_ << '\n';
    }
    out.close();
    if (!out || std::rename(temporary.c_str(), path.c_str()) != 0)
        throw std::runtime_error("Cannot write evaluation loop constraints: " + path);
}

void Backend::findLoopPairs(std::vector<LoopPair> &loop_vect) {
    auto& submaps = submap_manager_ptr_->getSubmaps();
    LoopClosureOptions options = backend_config_.loop_closure_options_;
    if (submaps.size() <2) {
       return;
    }
    for (int i = 0; i < static_cast<int>(submaps.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(submaps.size()); ++j) {
            if (j - i <= options.loop_min_gap_) {
                continue;
            }
            const Submap& si = submaps[i];
            const Submap& sj = submaps[j];

            Vec3d pi = si.centerPosition();
            Vec3d pj = sj.centerPosition();

            // double dist_xy = si.distanceXY(sj);
            // double dist_z = si.distanceZ(sj);
            double dist_xy = (pi.head<2>() - pj.head<2>()).norm();
            double dist_z = std::abs(pi.z() - pj.z());
            double yaw_diff = si.yawDiffTo(sj);

            if (dist_xy > options.xy_thresh_) {
                continue;
            }

            // if (dist_z > options.z_thresh_) {
            //     continue;
            // }

            // if (yaw_diff > options.yaw_thresh_) {
            //     continue;
            // }

            double overlap = bboxOverlapRatioXY(si, sj);

            if (overlap < options. min_overlap_) {
                continue;
            }

            LoopPair pair;
            pair.id_ = loop_vect.size();
            pair.idx_i_ = i;
            pair.idx_j_ = j;
            pair.dist_xy_ = dist_xy;
            pair.dist_z_ = dist_z;
            pair.yaw_diff_ = yaw_diff;
            pair.T_i_j_init_ = si.relativePoseTo(sj);
            std::cout<< "Loop closure: "<< pair.idx_i_ << "vs "<< pair.idx_j_<<std::endl;
            std::cout<< "dixt_xy: " << dist_xy<<std::endl;
            std::cout<<"overlap: "<<  overlap<<std::endl;
            loop_vect.push_back(pair);
        }
    }
    std::sort(loop_vect.begin(), loop_vect.end(),  [](const LoopPair& a, const LoopPair& b) {
            return a.dist_xy_ < b.dist_xy_;
        });
    return;
}

std::vector<int> Backend::nearbySubmapIds(int sid) const {
    LoopClosureOptions lconfig = backend_config_.loop_closure_options_;
    int radius = lconfig.submap_radius_;
    std::vector<int> ids;
    auto& submaps = submap_manager_ptr_->getSubmaps();
    int n = static_cast<int>(submaps.size());
    for (int d = -radius; d <= radius; ++d) {
        int id = sid + d;
        if (id >= 0 && id < n) {
            ids.push_back(id);
        }
    }
    return ids;
}

std::vector<LoopPair> Backend::makeSubmapPairsFromKfPair(const std::shared_ptr<KeyFrame>& kf_i,
    const std::shared_ptr<KeyFrame>& kf_j) {
    std::vector<LoopPair> candidates;

    const auto& submaps = submap_manager_ptr_->getSubmaps();
    const auto& options = backend_config_.loop_closure_options_;

    auto ids_i = nearbySubmapIds(kf_i->submap_id_);
    auto ids_j = nearbySubmapIds(kf_j->submap_id_);

    for (int si : ids_i) {
        for (int sj : ids_j) {
            if (si == sj) continue;

            int a = std::min(si, sj);
            int b = std::max(si, sj);

            double overlap = bboxOverlapRatioXY(submaps[a], submaps[b]);
            if (overlap < 0.2) continue;

            Vec3d pi = submaps[a].T_w_s_init_.translation();
            Vec3d pj = submaps[b].T_w_s_init_.translation();

            double dist_xy = (pi.head<2>() - pj.head<2>()).norm();
            if (dist_xy > options.xy_thresh_) continue;

            LoopPair lp;
            lp.idx_i_ = a;
            lp.idx_j_ = b;
            lp.kf_idx_i_ = kf_i->id_;
            lp.kf_idx_j_ = kf_j->id_;
            lp.overlap_ratio_ = overlap;
            lp.dist_xy_ = dist_xy;

            candidates.push_back(lp);
        }
    }
    return candidates;
}

void Backend::addCurrentNdtEdges(const std::vector<PoseGraphOptimizer::Node>& node_vect,  std::vector<PoseGraphOptimizer::Edge>&edge_vect) {
    //be aware, the node should be in order
    auto& submaps = submap_manager_ptr_->getSubmaps();
    Mat6d ndt_info = Mat6d::Zero();
    ndt_info(0,0) = 100.0;
    ndt_info(1,1) = 100.0;
    ndt_info(2,2) = 100.0;

    ndt_info(3,3) = 50.0;
    ndt_info(4,4) = 50.0;
    ndt_info(5,5) = 100.0;

    for (int i=0; i+1 < node_vect.size(); ++i) {

        PoseGraphOptimizer::Node node_i = node_vect[i];
        PoseGraphOptimizer::Node node_j = node_vect[i+1];
        //sanity check
        if (node_j.id_ != node_i.id_+1) {
           std::cout<<"nodes are no in order!"<<std::endl;
        }
        Submap& sm = submaps[node_i.id_];
        Submap& sm_nxt = submaps[node_j.id_];
        SE3d T_init = sm.T_w_s_init_.inverse() * sm_nxt.T_w_s_init_;
        //auto res = runSubmapNdt(submaps[i], submaps[i+1], T_init);
        SE3d T_sm1_sm2 = T_init;
        //NdtEval eval = res.first;
        //sm_nxt.T_w_s_init_ = sm.T_w_s_init_ * T_sm1_sm2;
        std::cout<< "Ndt pair: "<<sm.id_ <<" vs "<<sm_nxt.id_<<std::endl;
        PoseGraphOptimizer::Edge ndt_edge;
        ndt_edge.id_i_ = sm.id_;
        ndt_edge.id_j_ = sm_nxt.id_;
        ndt_edge.T_i_j_ = T_sm1_sm2;
        ndt_edge.info_ = ndt_info;
        edge_vect.push_back(ndt_edge);
    }
}


LoopPair Backend::chooseBestSubmapPairFromKfPair(const std::shared_ptr<KeyFrame>& kf_i,
    const std::shared_ptr<KeyFrame>& kf_j) {
    auto& submaps = submap_manager_ptr_->getSubmaps();

    LoopPair best;
    best.kf_idx_i_ = kf_i->id_;
    best.kf_idx_j_ = kf_j->id_;

    double best_overlap = -1.0;

    auto ids_i = nearbySubmapIds(kf_i->submap_id_);
    auto ids_j = nearbySubmapIds(kf_j->submap_id_);

    for (int si : ids_i) {
        for (int sj : ids_j) {
            if (si == sj) continue;

            double overlap =
                bboxOverlapRatioXY(submaps[si], submaps[sj]);

            if (overlap > best_overlap) {
                best_overlap = overlap;
                best.idx_i_ = si;
                best.idx_j_ = sj;
                best.overlap_ratio_ = overlap;

                Vec3d pi = submaps[si].T_w_s_init_.translation();
                Vec3d pj = submaps[sj].T_w_s_init_.translation();
                best.dist_xy_ =
                    (pi.head<2>() - pj.head<2>()).norm();
            }
        }
    }
    return best;
}

void Backend::findLoopPairsViaKfs(std::vector<LoopPair>& loop_vect) {
    std::vector<std::shared_ptr<KeyFrame>> all_kfs;
    LoopClosureOptions options = backend_config_.loop_closure_options_;

    for (auto& it : key_frames_) {
            if (it.second) all_kfs.push_back(it.second);
    }

    for (int i = 0; i < static_cast<int>(all_kfs.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(all_kfs.size()); ++j) {
            auto& ki = all_kfs[i];
            auto& kj = all_kfs[j];

            if (!ki || !kj) {
               continue;
            }
            if (std::abs(static_cast<int>(ki->id_) -
                         static_cast<int>(kj->id_)) <  options.loop_min_kf_gap_) {
                continue;
            }
            Vec3d pi = ki->scd_opti_pose_.translation();
            Vec3d pj = kj->scd_opti_pose_.translation();

            double dist_xy = (pi.head<2>() - pj.head<2>()).norm();
            double dist_z = std::abs(pi.z() - pj.z());

            if (dist_xy > options.xy_thresh_ || dist_z > options.z_thresh_) {
                continue;
            }

            if(ki->submap_id_ == -1 || kj->submap_id_ == -1 ) {
               continue;
            }

            std::vector<LoopPair> loop_pairs = makeSubmapPairsFromKfPair(ki, kj);
            for (const auto& lp : loop_pairs) {
                if (lp.overlap_ratio_ >options.min_overlap_) {
                    loop_vect.push_back(lp);
                }

            }
        }
    }
}

void Backend::selectSparseLoopPairs(std::vector<LoopPair>&in_vect, std::vector<LoopPair>&out) {

    LoopClosureOptions options = backend_config_.loop_closure_options_;
    out.clear();

    std::vector<std::vector<LoopPair>>events;
    groupLoopEvents(in_vect, events);

    for (auto& event : events) {
        LoopPair best = chooseBestInEvent(event);
        out.push_back(best);
        std::cout << "[Loop Event] candidates=" << event.size()
                 << " choose submap " << best.idx_i_ << " vs " << best.idx_j_
                 << " kf " << best.kf_idx_i_ << " vs " << best.kf_idx_j_
                 << " overlap=" << best.overlap_ratio_
                 << " dist=" << best.dist_xy_
                 << " score=" << preScoreLoopPair(best)
                 << std::endl;
    }
    std::cout << "[Loop Filter] raw=" << in_vect.size()
              << " events=" << events.size()
              << " selected=" << out.size()
              << std::endl;
}

void Backend::clearKfsLoop() {
    for (auto& it : key_frames_) {
         auto& kf = it.second;
         if (!kf) return;
         kf->is_loop_closure_ = false;
         kf->loop_role_ = 0;
    }
}

void Backend::markLoopClosureKfs(const std::vector<LoopPair>& loop_pairs) {
    auto& submaps = submap_manager_ptr_->getSubmaps();
    for (const auto& pair : loop_pairs) {
        if (pair.idx_i_ < 0 || pair.idx_j_ < 0) {
            continue;
        }

        if (pair.idx_i_ >= static_cast<int>(submaps.size()) ||
            pair.idx_j_ >= static_cast<int>(submaps.size())) {
            continue;
            }

        for (auto& kf : submaps[pair.idx_i_].key_frames_) {
            if (kf) {
                kf->is_loop_closure_ = true;
                kf->loop_role_ = 1;
            }
        }

        for (auto& kf : submaps[pair.idx_j_].key_frames_) {
            if (kf) {
                kf->is_loop_closure_ = true;
                kf->loop_role_ = 2;
            }
        }
    }
}

void Backend::runLoopRegistration(const std::vector<LoopPair>&pair_vect, std::vector<LoopConstraint>& loop_edges ) {
   //get submaps
   auto& submaps = submap_manager_ptr_->getSubmaps();
    //info_
   Mat6d loop_info  = Mat6d::Zero();
    loop_info(0,0) = 50.0;
    loop_info(1,1) = 50.0;
    loop_info(2,2) = 30.0;
    loop_info(3,3) = 20.0;
    loop_info(4,4) = 20.0;
    loop_info(5,5) = 50.0;
   //run submap lv ndt, check validility
   for (const auto& pair : pair_vect) {
       int i = pair.idx_i_;
       int j = pair.idx_j_;

       if (i < 0 || j < 0) continue;
       if (i >= static_cast<int>(submaps.size())) continue;
       if (j >= static_cast<int>(submaps.size())) continue;

       Submap& sm_i = submaps[i];
       Submap& sm_j = submaps[j];

       SE3d T_i_j_init =
            sm_i.T_w_s_opti_.inverse() * sm_j.T_w_s_opti_;

       auto res = runSubmapNdt(sm_i, sm_j, T_i_j_init);
       SE3d T_i_j_ndt = res.second;
       NdtEval eval = res.first;
       //remove it
       sm_i.T_w_s_opti_ = sm_i.T_w_s_init_;
       sm_j.T_w_s_opti_ = sm_i.T_w_s_opti_ * T_i_j_ndt;

       double trans_delta =
         (T_i_j_ndt.translation() - T_i_j_init.translation()).norm();

       auto getYaw = [](const SE3d& T) -> double {
           const Eigen::Matrix3d R = T.so3().matrix();
           // yaw around Z
           return std::atan2(R(1, 0), R(0, 0));
       };

       auto normalizeAngle = [](double a) -> double {
           while (a > M_PI)  a -= 2.0 * M_PI;
           while (a < -M_PI) a += 2.0 * M_PI;
           return a;
       };

       double yaw_init = getYaw(T_i_j_init);
       double yaw_ndt  = getYaw(T_i_j_ndt);
       double yaw_delta_deg =
           std::abs(normalizeAngle(yaw_ndt - yaw_init)) * 180.0 / M_PI;

       LoopConstraint lc;
       lc.idx_i_ = i;
       lc.idx_j_ = j;
       lc.T_i_j_ = T_i_j_ndt;
       lc.info_ = loop_info;
       lc.trans_delta_ = trans_delta;
       lc.yaw_delta_deg_ = yaw_delta_deg;
       lc.valid_ = true;

       loop_edges.push_back(lc);

       std::cout << "[LoopReg] accept "
                 << i << " " << j
                 << " trans_delta=" << trans_delta
                 << " yaw_delta=" << yaw_delta_deg
                 << std::endl;
   }
}
double Backend::preScoreLoopPair(const LoopPair& p)const {
    return 3.0 * p.overlap_ratio_
         - 0.05 * p.dist_xy_;
}

bool Backend::sameLoopEvent(const LoopPair& a,
                            const LoopPair& b,
                            int radius_i,
                            int radius_j) const {
    int a_i= static_cast<int>(a.idx_i_);
    int a_j= static_cast<int>(a.idx_j_);
    int b_i= static_cast<int>(b.idx_i_);
    int b_j= static_cast<int>(b.idx_j_);
    return abs( a_i- b_i) <= radius_i &&
           abs(a_j- b_j) <= radius_j;
}

void Backend::groupLoopEvents(
    const std::vector<LoopPair>& raw,
    std::vector<std::vector<LoopPair>>& events)
{
    //group tge pair that's  belongs to the same group
    LoopClosureOptions options = backend_config_.loop_closure_options_;
    events.clear();

    for (const auto& c : raw) {
        bool inserted = false;

        for (auto& event : events) {
            // compare with event representative
            const auto& rep = event.front();

            if (sameLoopEvent(c, rep, options.min_event_idx_gap_, options.min_event_idx_gap_)) {
                event.push_back(c);
                inserted = true;
                break;
            }
        }
        if (!inserted) {
            events.push_back({c});
        }
    }
}

LoopPair Backend::chooseBestInEvent(const std::vector<LoopPair>& event) {
    LoopPair best = event.front();
    double best_score = preScoreLoopPair(best);

    for (const auto& c : event) {
        double s = preScoreLoopPair(c);

        if (s > best_score) {
            best_score = s;
            best = c;
        }
    }
    return best;
}

void Backend::runSubmapRegistration(std::vector<LoopConstraint>& loop_edges) {
    auto& submaps =  submap_manager_ptr_->getSubmaps();
    if (submaps.size() <2) {
        std::cout<<"Not enogh submap pairs"<<std::endl;
    }
    //Todo: put it in config later
    Mat6d ndt_info = Mat6d::Zero();
    ndt_info(0,0) = 100.0;
    ndt_info(1,1) = 100.0;
    ndt_info(2,2) = 100.0;

    ndt_info(3,3) = 50.0;
    ndt_info(4,4) = 50.0;
    ndt_info(5,5) = 100.0;

    for(int i=0 ;i+1<submaps.size(); i++) {
        Submap& sm = submaps[i];
        Submap& sm_nxt = submaps[i+1];
        SE3d T_init = sm.T_w_s_init_.inverse() * sm_nxt.T_w_s_init_;
        std::cout<< "Run submap lv ndt: " <<i << "vs" << i+1 << std::endl;


        auto res = runSubmapNdt(submaps[i], submaps[i+1], T_init);
        SE3d T_sm1_sm2 = res.second;
        NdtEval eval = res.first;
        sm_nxt.T_w_s_opti_ = sm.T_w_s_init_ * T_sm1_sm2;
        //SE3d T_sm1_sm2 = SE3d();
        //node
        LoopConstraint lc;
        lc.idx_i_ = sm.id_;
        lc.idx_j_ = sm.id_;
        lc.T_i_j_ = T_sm1_sm2;
        lc.info_ = ndt_info;
        lc.valid_ = true;
        loop_edges.push_back(lc);
    }
}

void Backend::runTest(loop_callback callback) {
   //itreate  10 kf each time
   int cur_cnt =0;
   for (auto &it : key_frames_) {
      cur_cnt++;
      auto& kf = it.second;
      std::cout<< kf->id_ <<std::endl;
      if (cur_cnt ==100) {
         callback(cur_cnt);
          cur_cnt = 0;
      }


   }
   return;
}


// Fixed measurements isolate the effect of GNSS from loop detection or re-registration.
void Backend::runFixedLoopGnssXY(loop_callback callback) {
    if(key_frames_.empty()) throw std::runtime_error("No keyframes for GNSS trial");
    clearKfsLoop();
    evaluation_loops_.clear();
    std::ifstream input(backend_config_.fixed_loop_path_);
    if(!input) throw std::runtime_error("Cannot read fixed loop constraints");
    std::string line;
    while(std::getline(input,line)) {
        if(line.empty() || line[0]=='#') continue;
        std::istringstream row(line); LoopPair loop; Vec3d t; double x,y,z,w;
        if(!(row>>loop.kf_idx_i_>>loop.kf_idx_j_>>t.x()>>t.y()>>t.z()>>x>>y>>z>>w>>loop.ndt_score_))
            throw std::runtime_error("Malformed fixed loop constraint");
        Eigen::Quaterniond q(w,x,y,z);
        if(!t.allFinite() || !q.coeffs().allFinite() || std::abs(q.norm()-1)>1e-4 ||
           !key_frames_.count(loop.kf_idx_i_) || !key_frames_.count(loop.kf_idx_j_))
            throw std::runtime_error("Invalid fixed loop constraint");
        q.normalize();loop.T_i_j_init_=SE3d(q,t);evaluation_loops_.push_back(loop);
    }
    if(evaluation_loops_.empty()) throw std::runtime_error("No fixed loop constraints");
    std::vector<PoseGraphOptimizer::Node> nodes;
    std::vector<PoseGraphOptimizer::Edge> edges;
    std::map<size_t,SE3d> initial;
    std::set<size_t> selected;
    std::shared_ptr<KeyFrame> previous;
    double last_gnss=-INFINITY;
    for(const auto& entry:key_frames_) {
        const auto& kf=entry.second;
        PoseGraphOptimizer::Node node; node.id_=entry.first;node.t_=kf->time_;node.pose_init_=kf->loop_opti_pose_;
        node.has_gps_=kf->rtk_valid_ && kf->rtk_pose_.translation().head<2>().allFinite() && kf->time_-last_gnss>=backend_config_.gnss_min_interval_sec_;
        if(node.has_gps_) {node.gps_pos_=Vec3d(kf->rtk_pose_.translation().x(),kf->rtk_pose_.translation().y(),0);last_gnss=kf->time_;selected.insert(entry.first);}
        nodes.push_back(node);initial[entry.first]=node.pose_init_;
        if(previous) {PoseGraphOptimizer::Edge edge;edge.id_i_=previous->id_;edge.id_j_=kf->id_;
            edge.T_i_j_=previous->lidar_pose_neu_.inverse()*kf->lidar_pose_neu_;edge.info_=1000*Mat6d::Identity();edges.push_back(edge);}
        previous=kf;
    }
    if(selected.empty()) throw std::runtime_error("No usable GNSS constraints after coverage audit");
    for(const auto& loop:evaluation_loops_) {PoseGraphOptimizer::Edge edge;edge.id_i_=loop.kf_idx_i_;edge.id_j_=loop.kf_idx_j_;
        edge.T_i_j_=loop.T_i_j_init_;edge.info_=10*Mat6d::Identity();edge.is_loop_=true;edges.push_back(edge);}
    PoseGraphOptimizer optimizer(OptimizationStage::GNSS_XY_PLANAR);
    Mat3d information=Mat3d::Zero();information(0,0)=information(1,1)=backend_config_.gnss_information_xy_;
    optimizer.setGpsInfo(information);optimizer.setNodes(nodes);optimizer.setEdges(edges);optimizer.setStopRequested(stop_requested_);
    std::cout<<"[GNSS XY] Joint graph: "<<nodes.size()<<" nodes, "<<evaluation_loops_.size()<<" fixed loops, "<<selected.size()
             <<" horizontal factors; information="<<backend_config_.gnss_information_xy_<<"; height disabled; baseline Z/roll/pitch preserved; Huber delta=2\n";
    if(!optimizer.optimize(30)) {if(stopped())return;throw std::runtime_error("XY-only joint optimization failed");}
    std::map<size_t,SE3d> optimized;optimizer.getOptimizedPoses(optimized);
    if(optimized.size()!=key_frames_.size()) throw std::runtime_error("Incomplete XY-only optimization");
    for(const auto& pose:optimized) key_frames_.at(pose.first)->loop_opti_pose_=pose.second;
    if(!backend_config_.gnss_diagnostics_path_.empty()) {
        std::ofstream log(backend_config_.gnss_diagnostics_path_);
        if(!log)throw std::runtime_error("Cannot open GNSS diagnostics");
        log<<"kf_id,time,gnss_valid,selected,initial_xy_residual_m,final_xy_residual_m,pose_change_m,z_change_m\n"<<std::setprecision(17);
        for(const auto& entry:key_frames_) {
            const auto& kf=entry.second;const auto a=initial.at(entry.first).translation(),b=kf->loop_opti_pose_.translation();
            log<<entry.first<<','<<kf->time_<<','<<kf->rtk_valid_<<','<<selected.count(entry.first)<<',';
            if(kf->rtk_valid_)log<<(a-kf->rtk_pose_.translation()).head<2>().norm()<<','<<(b-kf->rtk_pose_.translation()).head<2>().norm();else log<<"nan,nan";
            log<<','<<(b-a).norm()<<','<<b.z()-a.z()<<'\n';
        }
        log.close();if(!log)throw std::runtime_error("Cannot complete GNSS diagnostics");
    }
    // Inspect the reused pairs against the same final joint solution.
    for(const auto& loop:evaluation_loops_) {
        if(stopped())return;
        markLoopSlidingWindow(loop.kf_idx_i_,loop.kf_idx_j_);
        if(callback)callback(static_cast<int>(key_frames_.size()));
    }
}
