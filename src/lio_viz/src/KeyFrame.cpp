//
// Created by chuchu on 3/10/26.
//
#include "KeyFrame.h"
#include "../include/KeyFrame.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "DataType.h"

/*
SE3d lidar_pose_;
SE3d lidar_pose_neu_;
SE3d rtk_pose_;
SE3d fst_opti_pose_;
SE3d scd_opti_pose_;
*/

void KeyFrame::write(std::ostream &os) {
    auto saveSE3 = [](std::ostream& ss, SE3d pose)->void{
        auto q = pose.so3().unit_quaternion();
        Vec3d t = pose.translation();
        //std::cout<<"write: "<<t[0] << " " << t[1] << " " << t[2] <<std::endl;
        ss << t[0] <<" "<< t[1]<<" "<<t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
    };
    os<< id_ << " "<< cloud_path_<< std::setprecision(18)<<" "<< time_ <<" "<<rtk_heading_valid_<<" "<<rtk_valid_ << " "
       << rtk_inlier_ << " ";
    saveSE3(os, lidar_pose_);
    saveSE3(os, lidar_pose_neu_);
    saveSE3(os, rtk_pose_);
    saveSE3(os, fst_opti_pose_);
    saveSE3(os, scd_opti_pose_);
    os<<std::endl;
}


void KeyFrame::read(std::istream &is) {
   auto load_SE3 = [](std:: istream& ss) -> SE3d {
     SE3d pose = SE3d();
     std::vector<double>out(7);
     ss>> out[0] >> out[1] >> out[2] >> out[3] >> out[4] >> out[5] >> out[6];
     //std::cout<<"read: " << out[0] << out[1] << out[2] <<out[3]<< out[4] << out[5] << out[6]<< std::endl;
     Eigen::Quaterniond q(out[6], out[5], out[4], out[3]);
     Vec3d t(out[0], out[1], out[2]);
     return SE3d(q, t);
   };
   is>> id_ >>  cloud_path_>> time_ >> rtk_heading_valid_>>rtk_valid_>>rtk_inlier_;
   lidar_pose_ = load_SE3(is);
   lidar_pose_neu_ = load_SE3(is);
   rtk_pose_ = load_SE3(is);
   fst_opti_pose_ = load_SE3(is);
   scd_opti_pose_ = load_SE3(is);
}

void writeKeyFramesToFile(const std::string &path, const std::map<size_t,std::shared_ptr<KeyFrame>>& kf_map) {
   std::ofstream os(path,  std::ios::out|std::ios::trunc);
   if (!os.is_open()) {
      return;
   }
   for(auto& it : kf_map) {
     it.second->write(os);
   }
   os.close();
}

void loadKeyFrames(const std::string &path, std::map<size_t, std::shared_ptr<KeyFrame>> &kf_map) {
    std::ifstream out(path, std::ios::app);
    if (!out) {
        std::cerr << "Failed to open file\n";
        return;
    }

    while(!out.eof()) {
        std::string line;
        std::getline(out, line);

        if(line.empty()) {
            break;
        }
        std::stringstream ss;
        ss<<line;
        auto kf = std::make_shared<KeyFrame>();
        kf->read(ss);
        std::cout<<kf->id_<<std::endl;
        if (!loadPointCloudFromFile(kf->cloud_path_, kf->cloud_ptr_)) {
             std::cerr << "Failed to load keypoints from file" << kf->cloud_path_<<"\n";
             continue;
        }
        kf->cloud_size_ = kf->cloud_ptr_->size();
        kf_map.emplace(kf->id_, kf);
    }
}

void  KeyFrameQueue::push(std::shared_ptr<KeyFrame> kf) {
   {
      std::lock_guard<std::mutex>lock(mtx_);
      q_.push(kf);
   }
   cv_.notify_one();
}

bool KeyFrameQueue::wait_and_pop(std::shared_ptr<KeyFrame>& kf) {
   std::unique_lock<std::mutex> lock(mtx_);
   cv_.wait(lock, [&]() {
      return !q_.empty() || stop_requested_;
   });
   if(q_.empty()) {
     return false;
   }
   kf = q_.front();
   q_.pop();
   return true;
}

void KeyFrameQueue::stop() {
    {
       std::lock_guard<std::mutex> lock(mtx_);
       stop_requested_ =true;
    }
}



