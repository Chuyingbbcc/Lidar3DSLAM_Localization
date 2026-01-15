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

IncNDTOdometry::IncNDTOdometry(LidarOdometry::LoOption op): LidarOdometry(op) {
  inc_ndt_ = std::make_shared<IncNDT>(op.inc_opt_);
}

bool LidarOdometry::IsKeyFrame(const SE3f& cur_pose) {
  if(cnt_frame_ <10) {
    return true;
  }
  SE3f delta = last_pose_.inverse() * cur_pose;
  return  delta.translation().norm() > opt_.kf_distance_ ||
               delta.so3().log().norm() > opt_.kf_angle_degree_ * M_1_PI/180.0f;
}

void IncNDTOdometry::AddCloud(std::shared_ptr<PointCloud>pointcloud, SE3f& pose,bool use_guess) {
  if(is_first_frame_) {
    last_pose_ = SE3f();
    inc_ndt_->AddCloud(pointcloud);
    is_first_frame_ = false;
    return;
  }
  SE3f guess = SE3f();
  inc_ndt_->SetSourceCloud(pointcloud);
  if (estimated_vec_.size() <2) {
    inc_ndt_->Align(guess);
  }
  else{
    if (use_guess) {
      SE3f t1 = estimated_vec_[estimated_vec_.size()-1];
      SE3f t2 = estimated_vec_[estimated_vec_.size()-2];
      SE3f delta = t2.inverse() * t1; // t2 * delta -> t1
      guess = t1* delta;
    }else {
      guess = pose;
    }
    inc_ndt_->Align(guess);
  }
  pose = guess;
  estimated_vec_.emplace_back(pose);

  //transform cur frame
  transform(pointcloud, pose);
  if(IsKeyFrame(pose)) {
    inc_ndt_->AddCloud(pointcloud);
    last_pose_ = pose;
    if(save_in_submap_) {
      SaveSubMap(pointcloud);
    }
  }
  cnt_frame_++;
}

void IncNDTOdometry::SaveSubMap(const std::shared_ptr<PointCloud>point_cloud) {
  //TODO: implement later
  std::cout<<"SaveSubMap"<<std::endl;
  std::string out_path = out_dir + "/cloud.bin";
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
  std::cout<< "Written "<< total_size<< " of " <<n<<"points"<<std::endl;
  //open the file handler
  //write binary to the file
}