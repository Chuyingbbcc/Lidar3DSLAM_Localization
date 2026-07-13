//
// Created by chuchu on 12/30/25.
//
#include "lidar_odometry.h"
#include "Align.h"

#include "common.h"
#include "DataType.h"


#include <iostream>
#include <ostream>
#include <fstream>

#include "../include/DataType.h"

IncNDTOdometry::IncNDTOdometry(const LidarOdometry::LoOption& op): LidarOdometry(op) {
  inc_ndt_ = std::make_shared<IncNDT>(op.inc_opt_);
}

bool LidarOdometry::IsKeyFrame(const SE3d& cur_pose) {
  if(cnt_frame_ <10) {
    return true;
  }
  SE3d delta = last_pose_.inverse() * cur_pose;
  return  delta.translation().norm() > opt_.kf_distance_ ||
               delta.so3().log().norm() > opt_.kf_angle_degree_ * M_1_PI/180.0f;
}

size_t IncNDTOdometry::AddCloud(std::shared_ptr<PointCloud>& pointcloud, SE3d& pose,bool use_guess) {
  if(is_first_frame_) {
    last_pose_ = SE3d();
    inc_ndt_->AddCloud(pointcloud);
    is_first_frame_ = false;
    key_frame_idx_++;
    return key_frame_idx_;
  }
  std::cout<<"----------------------------------------------"<<std::endl;
  std::cout<<"Frame Id: " <<  cnt_frame_ <<std::endl;
  auto isReasonableDelta = [&](SE3d& delta)->bool {
    double trans_dist = delta.translation().norm();
    double rot_angle = delta.so3().log().norm();

    const double max_trans = 1.0;      // meters per frame, example
    const double max_rot   = 0.17;
    // radians per frame (~20 deg)
    if (trans_dist > max_trans || rot_angle > max_rot) {
      // std::cout<<cnt_frame_<<"th frame \n";
      // std::cout<<rot_angle<<" angle\n";
      // std::cout<<trans_dist<<" trans dist\n";
    }
    return trans_dist < max_trans && rot_angle < max_rot;
  };
  auto yawDeg = [](const SE3d& T) -> double {
    return std::atan2(
        T.rotationMatrix()(1, 0),
        T.rotationMatrix()(0, 0)) * 180.0 / M_PI;
  };
  auto isValidPredict= [&](const SE3d& predict, const SE3d& delta)-> bool {
    SE3d T_p_d = predict.inverse() * delta;
    Vec3d dt = T_p_d.translation();
    double dxy = std::sqrt(dt.x() * dt.x() + dt.y() * dt.y());
    double dyaw = abs(yawDeg(T_p_d));
    //Todo:: put these value to config
    if (dxy >= 1.0 || dyaw >= 0.3) {
      std::cout<<"input predict invalid!"<<std::endl;
      return false;
    }
    return true;
  };

  auto printDelta = [&](const std::string& tag,
                        const SE3d& before,
                        const SE3d& after) {
    SE3d delta = before.inverse() * after;

    Vec3d dt = delta.translation();

    double trans = dt.norm();
    double dxy = std::sqrt(dt.x() * dt.x() + dt.y() * dt.y());
    double dz = dt.z();

    double yaw = yawDeg(delta);
    double rot3d =
        delta.so3().log().norm() * 180.0 / M_PI;

    std::cout << "[LO Debug] frame " << cnt_frame_
              << " " << tag << "\n"
              << "  before t = "
              << before.translation().transpose() << "\n"
              << "  after  t = "
              << after.translation().transpose() << "\n"
              << "  dxy      = " << dxy << " m\n"
              << "  dz       = " << dz << " m\n"
              << "  trans    = " << trans << " m\n"
              << "  yaw      = " << yaw << " deg\n"
              << "----------------------------------"
              << std::endl;
  };

  SE3d guess = SE3d();
  inc_ndt_->SetSourceCloud(pointcloud);
  if (estimated_vec_.size() <2) {
    SE3d guess_before_align = guess;
    inc_ndt_->Align(guess);
    //printDelta("align correction", guess_before_align, guess);
  }
  else{
    SE3d t1 = estimated_vec_[estimated_vec_.size()-1];
    SE3d t2 = estimated_vec_[estimated_vec_.size()-2];
    SE3d delta = t2.inverse() * t1; // t2 * delta -> t1
    SE3d init_guess = t1* delta;
    //check if delta and input predict pose
    bool pose_valid = isValidPredict(init_guess, pose);
    if (use_guess || !pose_valid) {
      isReasonableDelta(delta);
      guess = init_guess;
    }else {
      guess = pose;
    }
    SE3d guess_before_align = guess;
    inc_ndt_->Align(guess);
    std::string use_guess_str =  use_guess? "true": "false";
    std::cout<< "Use Guess: " << use_guess_str << std::endl;
    printDelta("aligned vs delta", init_guess, guess);
    printDelta("prediction vs delta", pose, guess);
  }
  //print the prediction pr delta
  pose = guess;
  bool valid =true;
  if (estimated_vec_.size() >0) {
    SE3d dT = estimated_vec_.back().inverse() * pose;
    valid = isReasonableDelta(dT);
    if(!valid ) {
      std::cout<< cnt_frame_ << " not reasonable!" <<std::endl;
      // if (estimated_vec_.size() >2) {
      //   //use delta
      //   SE3d t1 = estimated_vec_[estimated_vec_.size()-1];
      //   SE3d t2 = estimated_vec_[estimated_vec_.size()-2];
      //   SE3d delta = t2.inverse() * t1; // t2 * delta -> t1
      //   pose = t1* delta;
      // }
    }
  }

  //transform cur frame
    estimated_vec_.emplace_back(pose);

    //inc_ndt_->AddCloud(pointcloud);
  if(IsKeyFrame(pose)) {
    transformCloud(pointcloud, pose);
    inc_ndt_->AddCloud(pointcloud);
    last_pose_ = pose;
    cnt_frame_++;
    key_frame_idx_++;
    return key_frame_idx_;
    std::cout<<"kf id: " <<key_frame_idx_<<std::endl;
  }
  cnt_frame_++;
  //std::cout<<"----------------------------------------------"<<std::endl;
  return 0;
}

void IncNDTOdometry::SaveSubMap(const std::shared_ptr<PointCloud>point_cloud) {
  //TODO: implement later
  std::cout<<"SaveSubMap"<<std::endl;
  std::string out_path = out_dir_ + "/cloud.bin";
  std::ofstream ofs(out_path.c_str(), std::ios::binary|std::ios::app);
  if(!ofs) {
    std::cerr<< "Can't open file" << std::endl;
    return;
  }
  //write in only one cloud file
  uint64_t n = point_cloud->size();
  for(int i=0; i< n; i++) {
    const auto& p = (*point_cloud)[i];
    ofs.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&p.intensity), sizeof(float));
    ofs.write(reinterpret_cast<const char*>(&p.t), sizeof(float));
  }
  uint64_t total_size = n*sizeof(float)*5;

  //std::cout<< "Written "<< total_size<< " of " <<n<<"points"<<std::endl;
  //open the file handler
  //write binary to the file
}