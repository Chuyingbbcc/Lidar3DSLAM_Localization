//
// Created by chuchu on 3/10/26.
//
//key frame is used to store pose and lidar as output of lio
#pragma once
#include <condition_variable>

#include "DataType.h"
#include "point_cloud_utils.h"
#include <string>
#include <map>
#include <queue>


class KeyFrame {
 public:
   KeyFrame() {
   }

   KeyFrame(double time, size_t id, const SE3d& lidar_pose, std::shared_ptr<PointCloud> cloud_ptr): time_(time), id_(id), lidar_pose_(lidar_pose), cloud_ptr_(std::move(cloud_ptr)) {};
   double time_;
   size_t id_ =-1;
   SE3d lidar_pose_;
   SE3d lidar_pose_neu_;
   SE3d rtk_pose_;
   SE3d fst_opti_pose_;
   SE3d scd_opti_pose_;

   bool rtk_heading_valid_ = false;
   bool rtk_valid_ = true;
   bool rtk_inlier_ = true;
   bool fst_opti_valid_ = false;
   std::string cloud_path_ ="path";
   size_t cloud_size_ = 0;
   std::shared_ptr<PointCloud> cloud_ptr_ =std::make_shared<PointCloud>();

   void write(std::ostream& os);
   void read (std::istream& is);

};

void writeToFile(const std::string &path, const std::map<size_t,std::shared_ptr<KeyFrame>>& kf_map);
void loadKeyFrames(const std::string& path, std::map<size_t,std::shared_ptr<KeyFrame>>& kf_map);
//

class KeyFrameQueue {
public:
    void push(std::shared_ptr<KeyFrame> kf);
    bool wait_and_pop(std::shared_ptr<KeyFrame>& kf);
    void stop();
private:
    std::queue<std::shared_ptr<KeyFrame>>q_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_requested_ = false;
};
