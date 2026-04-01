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
  auto isReasonableDelta = [&](SE3d& delta)->bool {
    double trans_dist = delta.translation().norm();
    double rot_angle = delta.so3().log().norm();

    const double max_trans = 1.0;      // meters per frame, example
    const double max_rot   = 0.17;
    // radians per frame (~20 deg)
    if (trans_dist > max_trans || rot_angle > max_rot) {
      std::cout<<cnt_frame_<<"th frame \n";
      std::cout<<rot_angle<<" angle\n";
      std::cout<<trans_dist<<" trans dist\n";
    }

    return trans_dist < max_trans && rot_angle < max_rot;
  };
  SE3d guess = SE3d();
  inc_ndt_->SetSourceCloud(pointcloud);
  if (estimated_vec_.size() <2) {
    inc_ndt_->Align(guess);
  }
  else{
    if (use_guess) {
      SE3d t1 = estimated_vec_[estimated_vec_.size()-1];
      SE3d t2 = estimated_vec_[estimated_vec_.size()-2];
      SE3d delta = t2.inverse() * t1; // t2 * delta -> t1
      isReasonableDelta(delta);
      guess = t1* delta;
    }else {
      guess = pose;
    }
    inc_ndt_->Align(guess);
  }
  pose = guess;
  bool valid =true;
  if (estimated_vec_.size() >0) {
    SE3d dT = estimated_vec_.back().inverse() * pose;
    valid = isReasonableDelta(dT);
    if(!valid ) {
      std::cout<< cnt_frame_ << " not reasonable!" <<std::endl;
    }
  }

  //transform cur frame
    estimated_vec_.emplace_back(pose);
    transform(pointcloud, pose);
    inc_ndt_->AddCloud(pointcloud);
  if(IsKeyFrame(pose)) {
    last_pose_ = pose;
    cnt_frame_++;
    key_frame_idx_++;
    return key_frame_idx_;
  }
  cnt_frame_++;
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