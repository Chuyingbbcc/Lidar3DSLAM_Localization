//
// Created by chuchu on 6/1/26.
//
#include "KeyFrame.h"
#include "Slam/Backend.h"

#include <iostream>
#include <memory>

#include  "Submap.h"
#include "Align.h"
#include "common.h"
#include <yaml-cpp/yaml.h>
#include "RouteAlign.h"


bool Backend::loadBackendConfig(const std::string &init_path) {
  try {
        YAML::Node root = YAML::LoadFile(init_path);

        if (!root["backend"]) {
            std::cerr << "Missing 'backend' section in yaml: " << init_path << std::endl;
            return false;
        }

        YAML::Node cfg = root["backend"];

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
      backend_config_.inc_ndt_options_ = backend_config_.inc_ndt_options_;
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

Backend::Backend(const std::string &init_path) {
  loadBackendConfig(init_path);
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
  loadKeyFrames(backend_config_.in_kf_info_path_, key_frames_);
  std::cout<< "be kfs size: " <<key_frames_.size() << std::endl;
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
    transformCloud(cloud, kf->lidar_pose_neu_);
    fixed_ndt.AddCloud(cloud);
  }

  //refine each kf against fixed map
  for (auto& kf: submap.key_frames_) {
    SE3d init_pose= kf->lidar_pose_neu_;
    double score_before = fixed_ndt.computeScore(kf->lidar_pose_neu_);
    fixed_ndt.SetSourceCloud(kf->cloud_ptr_);
    bool ok  = fixed_ndt.Align(init_pose);
    double score_after = fixed_ndt.computeScore(init_pose);

    bool improved = std::isfinite(score_before) &&
    std::isfinite(score_after) &&
    score_after < score_before * 0.98;

    if (ok && improved) {
      kf->fst_opti_pose_ = init_pose;
    }
    else {
      kf->fst_opti_pose_ = kf->lidar_pose_neu_;
      std::cout << "keep use old pose" <<std::endl;
      std::cout<< "Score before: " << score_before << std::endl;
      std::cout << "Score after: " << score_after << std::endl;
      std::cout << "------------------------------------"<<std::endl;
    }
    SE3d delta = kf->lidar_pose_neu_.inverse() * kf->fst_opti_pose_;
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
  //store pose used for 2nd opti
  submap.pose_init_ = submap.key_frames_.front()->fst_opti_pose_;
  submap.pose_optimized_ = submap.pose_init_;
}

void Backend::runLevel1Optimization() {
  auto& submaps =  submap_manager_ptr_ ->getSubmaps();
  for (auto& submap : submaps) {
    optimizeOneSubmapLevel10(submap);
  }
}

void Backend::runLevel2Optimization() {
    std::vector<PoseGraphOptimizer::Node>node_vect;
    buildSubmapGraphNodes(node_vect);
    std::cout
    << "\n========== Lv2 Input ==========\n";

    for (const auto& node : node_vect) {

        std::cout
            << "[Node] "
            << node.id_
            << " pose="
            << node.pose_init_.translation().transpose();

        if (node.has_gps_) {
            double diff =
                (node.pose_init_.translation() -
                 node.gps_pos_).norm();

            std::cout
                << " gps="
                << node.gps_pos_.transpose()
                << " diff="
                << diff;
        }

        std::cout << std::endl;
    }
    PoseGraphOptimizer optimizer;
    optimizer.setNodes(node_vect);
    bool ok = optimizer.optimize(20);
    if(!ok) {
        std::cout << "[Lv2] optimization failed, use passthrough.\n";
        applyLevel20Correction();
        return;
    }
    std::map<size_t,SE3d> opt_poses;
    optimizer.getOptimizedPoses(opt_poses);
    std::cout<< "\n========== Lv2 Result ==========\n";
    //update back to the submap
    for (auto& sm : submap_manager_ptr_->getSubmaps()) {
        auto it = opt_poses.find(sm.id_);
        if (it != opt_poses.end()) {
            sm.pose_optimized_ = it->second;
            Vec3d t_init = sm.pose_init_.translation();
            Vec3d t_opt  = sm.pose_optimized_.translation();

            Vec3d move = t_opt - t_init;

            SE3d rel = sm.pose_init_.inverse() * sm.pose_optimized_;

            double yaw_deg =
                std::atan2(
                    rel.rotationMatrix()(1, 0),
                    rel.rotationMatrix()(0, 0))
                * 180.0 / M_PI;

            std::cout << "[Lv2] Submap " << sm.id_
                      << "\n init = " << t_init.transpose()
                      << "\n opt  = " << t_opt.transpose()
                      << "\n move = " << move.transpose()
                      << "\n move_xy = " << move.head<2>().norm()
                      << "\n move_xyz = " << move.norm()
                      << "\n rel_yaw = " << yaw_deg << " deg"
                      << "\n----------------------\n";
            if (move.head<2>().norm() > 10.0 ||
    std::abs(yaw_deg) > 5.0) {
                std::cout<<"[Lv2 warning] change too large!"<<std::endl;
            }
            size_t mid = sm.key_frames_.size()/2;
            auto sm_gps_pos = sm.key_frames_[mid]->rtk_pose_.translation();
            double before =
             (sm.pose_init_.translation() - sm_gps_pos).norm();

            double after =
                (sm.pose_optimized_.translation() - sm_gps_pos).norm();

            std::cout << "[GPS Check] Submap " << sm.id_
                      << " before=" << before
                      << " after=" << after
                      << std::endl;

        }
    }
    applyLevel20Correction();
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
    std::vector<AlignedPair>ap_vect;
    for(auto it : key_frames_) {
      AlignedPair ap;
      auto kf = it.second;
      ap.timestamp_ = kf->time_;
      ap.p_odom_ = kf->lidar_pose_.translation();
      ap.p_rtk_ = kf->rtk_pose_.translation();
      ap_vect.emplace_back(ap);
    }
    //neu align
      Rigid2D align = RouteAlign::estimateRobustRigid2D(ap_vect, 2, 0.85, 10);
      double z_offset = RouteAlign::estimateMedianZOffset(ap_vect, align);

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
          auto kf = it.second;
          kf->lidar_pose_neu_ = convert2neu(kf->lidar_pose_);

          auto dist = (kf->lidar_pose_neu_.translation() - kf->rtk_pose_.translation()).norm();
          // std::cout<< "kf: " << kf->id_ << " dist: " << dist << std::endl;
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