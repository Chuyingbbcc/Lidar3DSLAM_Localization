//
// Created by chuchu on 12/12/25.
//
#include "mapping_node.h"
#include <iostream>
#include <thread>
#include <Eigen/Core>

#include "../include/KeyFrame.h"
#include "../include/RouteAlign.h"
#include "../include/Slam/Optimization.h"
#include "io/ros_io_offline.cpp"

using namespace std::chrono_literals;

MappingNode::MappingNode(): Node("mapping_node") {

this->declare_parameter<std::string>("mode", "mapping");
mode_ = this->get_parameter("mode").as_string();
std::cout<<"Current mode is: "<<mode_<<std::endl;

pub_ = create_publisher<std_msgs::msg::String>("pcd_path", 10);
frame_pub_ = create_publisher<lio_msgs::msg::FrameData>("frame_data", 10);
std::string init_path  = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/Config/config.yaml";
frontend_ptr_ = std::make_unique<Frontend>(init_path);
//run frontend

if (mode_  == "mapping") {
  slam_thread_ = std::thread([this]() {
    try {
      frontend_ptr_->init();
      frontend_ptr_->Run();
    }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Frontend exception: %s", e.what());
  }
  catch(...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown frontend exception.");
  }
  });
}
else if (mode_ == "replay") {
std::map<size_t, std::shared_ptr<KeyFrame>>kf_map;
loadKeyFrames("/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/kf_output.txt", kf_map);
for(auto it : kf_map) {
  replay_kf_q_.push(it.second);
}
}

else if (mode_ == "optimization") {
 //load key frames
  std::map<size_t, std::shared_ptr<KeyFrame>>kf_map;
  const std::string kf_path = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/kf_output.txt";
  loadKeyFrames(kf_path, kf_map);
  //convert kf tp nodes(I have to keep the order )
  std::vector<PoseGraphOptimizer::Node>node_vect;
  //convert to aligned pair
  std::vector<AlignedPair>ap_vect;
  for(auto it : kf_map) {
    PoseGraphOptimizer::Node node;
    AlignedPair ap;
    node.id_  = it.first;
    auto kf = it.second;
    ap.timestamp_ = kf->time_;
    ap.p_odom_ = kf->lidar_pose_.translation();
    ap.p_rtk_ = kf->rtk_pose_.translation();
    node.pose_init_ = kf->lidar_pose_;
    if (kf->rtk_valid_) {
      node.has_gps_ = true;
      node.gps_pos_ = kf->rtk_pose_.translation();
    }
    node_vect.emplace_back(node);
    ap_vect.emplace_back(ap);
  }
  //neu align
  Rigid2D align = RouteAlign::estimateRobustRigid2D(ap_vect,2,0.85, 10);
  double z_offset =RouteAlign::estimateMedianZOffset(ap_vect,align);
  //update lidar_pose_neu_ in kf
  auto convert2neu = [&](SE3d& in_pose)->SE3d {
    Mat3d r_align = RouteAlign::yawToRotation3D(align.yaw_);
    Vec3d t_align(align.t_.x(), align.t_.y(), z_offset);
    Mat3d R_new = r_align * in_pose.rotationMatrix();
    Vec3d t_new = r_align * in_pose.translation() + t_align;
    return SE3d(R_new,t_new);
  };
  for(auto& it: kf_map ) {
    auto kf= it.second;
    kf->lidar_pose_neu_ = convert2neu(kf->lidar_pose_);
    replay_kf_q_.push(it.second);
  }
  //test now
  //write kf but not pointcloud

 // //init optimizer
 // PoseGraphOptimizer optimizer;
 // optimizer.setNodes(node_vect);
 // optimizer.optimize(20);
 // std::map<size_t,SE3d>output_poses;
 // optimizer.getOptimizedPoses(output_poses);
 // //update the keyframe from here
 // for (auto& it : output_poses) {
 //   if(kf_map.find(it.first) == kf_map.end()) {
 //     continue;
 //   }
 //   kf_map[it.first]->fst_opti_pose_ = output_poses.at(it.first);
 // }


}

timer_ =this->create_wall_timer(
30ms,
std::bind(&MappingNode::on_timer, this)
);
 RCLCPP_INFO(this->get_logger(), "Mapping node publishing path: %s", pcd_path_.c_str());
}

MappingNode::~MappingNode() {
  RCLCPP_INFO(this->get_logger(), "Destructor start");

  if (timer_) {
    RCLCPP_INFO(this->get_logger(), "Cancelling timer");
    timer_->cancel();
  }

  if (frontend_ptr_) {
    RCLCPP_INFO(this->get_logger(), "Stopping frontend");
    frontend_ptr_->stop();
    RCLCPP_INFO(this->get_logger(), "Frontend stopped");
  }

  if (slam_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining slam thread");
    slam_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Slam thread joined");
  }

  RCLCPP_INFO(this->get_logger(), "Destructor done");
}

void MappingNode::publish_frame() {
  std::shared_ptr<KeyFrame>nxt_kf_ptr = nullptr;
  if (mode_ == "mapping") {
    nxt_kf_ptr = frontend_ptr_->takeNextKeyFrame();
 }
 else {
   if(replay_kf_q_.size() >0) {
      nxt_kf_ptr = replay_kf_q_.front();
      replay_kf_q_.pop();
   }
 }
 if (!nxt_kf_ptr) {
   return;
 }
 lio_msgs::msg::FrameData msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  msg.frame_id = nxt_kf_ptr->id_;              // change to your real field
  msg.cloud_path = nxt_kf_ptr->cloud_path_;    // change to your real field

  auto se3ToMsgPose= [](const SE3d& T, geometry_msgs::msg::Pose& pose)->void{
    const Vec3d t =
       T.translation();

    const Eigen::Quaterniond q(
        T.so3().matrix());

    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return;
  };
  /*geometry_msgs/Pose lidar_pose
    geometry_msgs/Pose rtk_pose
    geometry_msgs/Pose lidar_pose_neu*/
   se3ToMsgPose(nxt_kf_ptr->lidar_pose_, msg.lidar_pose);
   se3ToMsgPose(nxt_kf_ptr->lidar_pose_neu_, msg.lidar_pose_neu);
   se3ToMsgPose(nxt_kf_ptr->rtk_pose_, msg.rtk_pose);
   frame_pub_->publish(msg);
}

void MappingNode::on_timer(){
  if(!frontend_ptr_) {
    return;
  }
  publish_frame();
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  std::cout<<"map node shut down!";
  return 0;
}