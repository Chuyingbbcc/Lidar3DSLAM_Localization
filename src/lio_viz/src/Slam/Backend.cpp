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
#include <algorithm>   // std::sort, std::min, std::max
#include <set>         // std::set
#include <utility>     // std::pair

#include  "Submap.h"
#include "Align.h"
#include "common.h"
#include <yaml-cpp/yaml.h>
#include "RouteAlign.h"
#include "../../include/KeyFrame.h"


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
    double score_before = fixed_ndt.computeScore(kf->fst_opti_pose_);
    fixed_ndt.SetSourceCloud(kf->cloud_ptr_);
    bool ok  = fixed_ndt.Align(init_pose);
    double score_after = fixed_ndt.computeScore(init_pose);

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
    std::vector<PoseGraphOptimizer::Node>node_vect;
    std::vector<PoseGraphOptimizer::Edge>edge_vect;

    buildKfGraphNodesAndEdges(node_vect, edge_vect);
    std::cout
    << "\n========== Lv2 Input ==========\n"
    << "There are: " << node_vect.size() << " nodes\n";

    PoseGraphOptimizer optimizer(OptimizationStage::KF_RTK_OPTI);
    optimizer.setNodes(node_vect);
    optimizer.setEdges(edge_vect);
    bool ok = optimizer.optimize(20);
    if(!ok) {
        std::cout << "[Lv2] optimization failed, use passthrough.\n";
        //applyLevel20Correction();
        return;
    }
    std::map<size_t,SE3d> opt_poses;
    optimizer.getOptimizedPoses(opt_poses);
    std::cout<< "\n========== Lv2 Result ==========\n";
    //update back to Kfs
    for (auto& kf_it : key_frames_) {
        auto& kf = kf_it.second;
        auto it = opt_poses.find(kf->id_);
        if (it != opt_poses.end()) {
            kf->fst_opti_pose_ = it->second;

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
    for(auto& it: key_frames_){
        auto kf = it.second;
        auto id =  it.first;
        //node
        PoseGraphOptimizer::Node node;
        node.id_ = kf->id_;
        node.t_ = kf->time_;
        node.pose_init_ = kf->lidar_pose_neu_;
        node.gps_pos_ =kf->rtk_pose_.translation();
        node_vect.push_back(node);
        //edge
        const SE3d& Ti = key_frames_[id]->lidar_pose_neu_;
        if (key_frames_.find(id+1) == key_frames_.end()) {
           std::cout<<"This is the last node!" <<std::endl;
           return;
        }
        const SE3d& Tj = key_frames_[id + 1]->lidar_pose_neu_;
        PoseGraphOptimizer::Edge edge;
        edge.id_i_ = key_frames_[id]->id_;
        edge.id_j_ = key_frames_[id + 1]->id_;
        //Todo: might change to other pose later
        edge.T_i_j_ = Ti.inverse() * Tj;
        edge.info_ = odom_info;
        edge_vect.push_back(edge);
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
    int count =0;
    for(auto it : key_frames_) {
      //Todo : nnumber of lidars used for neu estimzte
      if (count >1500) {
        break;
      }
      AlignedPair ap;
      auto kf = it.second;
      ap.timestamp_ = kf->time_;
      ap.p_odom_ = kf->lidar_pose_.translation();
      ap.p_rtk_ = kf->rtk_pose_.translation();
      ap_vect.emplace_back(ap);
        count++;
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
          //std::cout<< "kf: " << kf->id_ << " dist: " << dist << std::endl;
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

SE3d Backend::runSubmapNdt(Submap& sm1, Submap& sm2, const SE3d& init_pose) {
    //Todo:might need to change to seperate config later
    std::cout<<backend_config_.inc_ndt_options_.voxel_size_ <<std::endl;
    IncNDT submap_ndt(backend_config_.inc_ndt_options_);
    submap_ndt.AddCloud(sm1.point_cloud_ptr_);
    SE3d pose= init_pose;
    // double score_before = submap_ndt.computeScore(init_pose);
    submap_ndt.SetSourceCloud(sm2.point_cloud_ptr_);
    bool ok  = submap_ndt.Align(pose);
    if(!ok) {
        std::cout<< "Inc_ndt failed, sm: "<< sm2.id_<<std::endl;
    }
    return pose;
}

double Backend::bboxOverlapRatioXY(const Submap& a, const Submap& b)const {
    double ax0, ay0, ax1, ay1;
    double bx0, by0, bx1, by1;

    if (!a.getWorldBboxXY(ax0, ay0, ax1, ay1)) {
        return 0.0;
    }

    if (!b.getWorldBboxXY(bx0, by0, bx1, by1)) {
        return 0.0;
    }

    double ix = std::max(
        0.0,
        std::min(ax1, bx1) - std::max(ax0, bx0));

    double iy = std::max(
        0.0,
        std::min(ay1, by1) - std::max(ay0, by0));

    double inter = ix * iy;

    double area_a = std::max(1e-6, (ax1 - ax0) * (ay1 - ay0));
    double area_b = std::max(1e-6, (bx1 - bx0) * (by1 - by0));

    return inter / std::min(area_a, area_b);
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
    runLoopRegistration(sparse_vect, loop_constraints);

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
    std::vector<int> ids;
    auto& submaps = submap_manager_ptr_->getSubmaps();
    int n = static_cast<int>(submaps.size());
    for (int d = -1; d <= 1; ++d) {
        int id = sid + d;
        if (id >= 0 && id < n) {
            ids.push_back(id);
        }
    }
    return ids;
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

            LoopPair lp  = chooseBestSubmapPairFromKfPair(ki, kj);
            if (lp.idx_i_ < 0 || lp.idx_j_ < 0) continue;
            if (lp.overlap_ratio_ < 0.2) continue;

            loop_vect.push_back(lp);
            std::cout << "[KF -> Submap Loop] "
             << "kf " << lp.kf_idx_i_ << " vs " << lp.kf_idx_j_
             << " => submap " << lp.idx_i_ << " vs " << lp.idx_j_
             << " overlap = " << lp.overlap_ratio_
             << " dist = " << lp.dist_xy_
             << std::endl;
        }
    }
}

void Backend::selectSparseLoopPairs(std::vector<LoopPair>&in_vect, std::vector<LoopPair>&out) {
    std::sort(in_vect.begin(), in_vect.end(),
        [](const LoopPair& a, const LoopPair& b) {
            if (std::abs(a.overlap_ratio_ - b.overlap_ratio_) > 1e-6)
                return a.overlap_ratio_ > b.overlap_ratio_;
            return a.dist_xy_ < b.dist_xy_;
        });

    LoopClosureOptions options = backend_config_.loop_closure_options_;
    out.clear();
    std::vector<LoopPair>res;
    for (const auto& c : in_vect) {
        if (c.overlap_ratio_ < options.min_overlap_) continue;
        if (c.dist_xy_ > options.xy_thresh_) continue;

        int i = std::min(c.idx_i_, c.idx_j_);
        int j = std::max(c.idx_i_, c.idx_j_);

        // skip adjacent / near-time pairs
        if (std::abs(i - j) < options.loop_min_gap_) continue;

        bool too_close_to_existing = false;

        for (const auto& e : res) {
            int ei = std::min(e.idx_i_, e.idx_j_);
            int ej = std::max(e.idx_i_, e.idx_j_);

            // same loop region
            if (std::abs(i - ei) < options.min_exist_idx_gap_ ||
                std::abs( i - ej) < options.min_exist_idx_gap_) {
                too_close_to_existing = true;
                break;
                }

            // also handle reversed nearby matching
            if (std::abs(j - ej) < options.min_exist_idx_gap_ ||
                std::abs(j - ei) < options.min_exist_idx_gap_) {
                too_close_to_existing = true;
                break;
                }
        }

        if (too_close_to_existing) continue;
        res.push_back(c);
    }
    std::sort(res.begin(), res.end(), [](const LoopPair& a, const LoopPair& b) {
        return a.idx_i_ < b.idx_i_;
    });

    for (int i= 0; i< res.size(); i++) {
       out.push_back(res[i]);
    }
    for (auto& c: out) {
      std::cout << "i: " << c.idx_i_ << "j: " << c.idx_j_ << std::endl;
    }
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
    clearKfsLoop();
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
       SE3d T_i_j_ndt = runSubmapNdt(sm_i, sm_j, T_i_j_init);

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

        SE3d T_sm1_sm2 = runSubmapNdt(submaps[i], submaps[i+1], T_init);
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