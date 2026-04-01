//
// Created by chuchu on 3/10/26.
//
//key frame is used to store pose and lidar as output of lio
#pragma once
#include "DataType.h"
#include "point_cloud_utils.h"
#include <string>
#include <map>


class KeyFrame {
 public:
   KeyFrame() {
   }

   KeyFrame(double time, size_t id, const SE3d& lidar_pose, std::shared_ptr<PointCloud> cloud_ptr): time_(time), id_(id), lidar_pose_(lidar_pose), cloud_ptr_(std::move(cloud_ptr)) {};
   double time_;
   size_t id_ =0;
   SE3d lidar_pose_;
   SE3d rtk_pose_;
   SE3d fst_opti_pose_;
   SE3d scd_opti_pose_;

   bool rtk_heading_valid_ = false;
   bool rtk_valid_ = true;
   bool rtk_inlier_ = true;
   std::string cloud_path_ ="path";
   size_t cloud_size_ = 0;
   std::shared_ptr<PointCloud> cloud_ptr_ =std::make_shared<PointCloud>();

   void write(std::ostream& os);
   void read (std::istream& is);

};

void writeToFile(const std::string &path, const std::unordered_map<size_t,std::shared_ptr<KeyFrame>>& kf_map);
void loadKeyFrames(const std::string& path, std::unordered_map<size_t,std::shared_ptr<KeyFrame>>& kf_map);
