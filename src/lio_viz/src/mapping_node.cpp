//
// Created by chuchu on 12/12/25.
//
#include "mapping_node.h"
#include "../include/mapping_node.h"


#include <iostream>
#include <thread>
#include <csignal>
#include <Eigen/Core>
#include <fstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include "Slam/Backend.h"

#include "../include/KeyFrame.h"
#include "../include/RouteAlign.h"
#include "../include/Slam/Optimization.h"
#include "io/ros_io_offline.cpp"

using namespace std::chrono_literals;
static double yawFromR(const Mat3d& R)
{
  return std::atan2(R(1, 0), R(0, 0));
}

static void printPoseDebug(
    size_t id,
    const SE3d& lidar_pose,
    const SE3d& lidar_pose_neu,
    const SE3d& rtk_pose)
{
  const Vec3d t_lidar = lidar_pose.translation();
  const Vec3d t_neu   = lidar_pose_neu.translation();
  const Vec3d t_rtk   = rtk_pose.translation();

  const double yaw_lidar = yawFromR(lidar_pose.rotationMatrix());
  const double yaw_neu   = yawFromR(lidar_pose_neu.rotationMatrix());
  const double yaw_rtk   = yawFromR(rtk_pose.rotationMatrix());

  const Vec3d err = t_neu - t_rtk;

  std::cout << std::fixed << std::setprecision(3)
            << "KF " << id << "\n"
            << "  lidar t      = " << t_lidar.transpose()
            << " yaw=" << yaw_lidar << "\n"
            << "  lidar_neu t  = " << t_neu.transpose()
            << " yaw=" << yaw_neu << "\n"
            << "  rtk t        = " << t_rtk.transpose()
            << " yaw=" << yaw_rtk << "\n"
            << "  neu-rtk err  = " << err.transpose()
            << " norm=" << err.norm()
            << "\n";
}

static std::atomic<bool> g_exit_requested{false};

void signalHandler(int sig) {
    g_exit_requested.store(true);
    // Ctrl+C, kill, Ctrl+Z all come here
    rclcpp::shutdown();
}

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
  writeVisualizationConfig(init_path, MapMode::frontend);
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
else if (mode_ == "rfront" || mode_ == "roptimization") {
if (mode_ == "rfront") {
 writeVisualizationConfig(init_path, MapMode::replay_frontend);
}
if (mode_ == "roptimization") {
  std::cout<<"write roptimization config"<<std::endl;
  writeVisualizationConfig(init_path, MapMode::replay_optimization);
}
  std::map<size_t, std::shared_ptr<KeyFrame>>kf_map;
  loadKeyFrames("/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/kf_output.txt", kf_map);
  for(auto it : kf_map) {
    replay_kf_q_.push(it.second);
  }
}

else if (mode_ == "optimization") {
 Backend backend = Backend(init_path);
 backend.buildSubmaps();
 backend.neuAlign();
 backend.runLevel1Optimization();
 //backend.runLevel2Optimization();
 const std::string kf_path = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/kf_output.txt";
 writeKeyFramesToFile(kf_path, backend.getKeyFrames());
 //load key frames
 //std::map<size_t, std::shared_ptr<KeyFrame>>kf_map;

 //  loadKeyFrames(kf_path, kf_map);
 //  //convert kf tp nodes(I have to keep the order )
 //  std::vector<PoseGraphOptimizer::Node>node_vect;
 //  //convert to aligned pair
 //  std::vector<AlignedPair>ap_vect;
 //  for(auto it : kf_map) {
 //    PoseGraphOptimizer::Node node;
 //    AlignedPair ap;
 //    node.id_  = it.first;
 //    auto kf = it.second;
 //    ap.timestamp_ = kf->time_;
 //    ap.p_odom_ = kf->lidar_pose_.translation();
 //    ap.p_rtk_ = kf->rtk_pose_.translation();
 //    node.pose_init_ = kf->lidar_pose_;
 //    if (kf->rtk_valid_) {
 //      node.has_gps_ = true;
 //      node.gps_pos_ = kf->rtk_pose_.translation();
 //    }
 //    node_vect.emplace_back(node);
 //    ap_vect.emplace_back(ap);
 //  }
 //  //neu align
 //    Rigid2D align = RouteAlign::estimateRobustRigid2D(ap_vect, 2, 0.85, 10);
 //    double z_offset = RouteAlign::estimateMedianZOffset(ap_vect, align);
 //
 //    Mat3d R_align = RouteAlign::yawToRotation3D(align.yaw_);
 //
 //    Vec3d t_align(
 //        align.t_.x(),
 //        align.t_.y(),
 //        z_offset);
 //
 //    auto convert2neu = [&](const SE3d& in_pose) -> SE3d {
 //        Mat3d R_new = R_align * in_pose.rotationMatrix();
 //        Vec3d t_new = R_align * in_pose.translation() + t_align;
 //
 //        return SE3d(R_new, t_new);
 //    };
 //
 //    for (auto& it : kf_map) {
 //        auto kf = it.second;
 //        kf->lidar_pose_neu_ = convert2neu(kf->lidar_pose_);
 //
 //        SE3d Tc_i = kf->lidar_pose_neu_ * kf->lidar_pose_.inverse();
 //
 //        double yaw_i = std::atan2(
 //            Tc_i.rotationMatrix()(1, 0),
 //            Tc_i.rotationMatrix()(0, 0)
 //        );
 //
 //        std::cout << "kf " << it.first
 //                  << " Tc_i yaw = " << yaw_i
 //                  << " t = " << Tc_i.translation().transpose()
 //                  << std::endl;
 //    }
  //writeKeyFramesToFile(kf_path, kf_map);
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
  if (mode_ == "optimization") {
    return;
  }
  else if (mode_ == "mapping") {
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
  };
  /*geometry_msgs/Pose lidar_pose
    geometry_msgs/Pose rtk_pose
    geometry_msgs/Pose lidar_pose_neu*/
   se3ToMsgPose(nxt_kf_ptr->lidar_pose_, msg.lidar_pose);
   se3ToMsgPose(nxt_kf_ptr->lidar_pose_neu_, msg.lidar_pose_neu);
   se3ToMsgPose(nxt_kf_ptr->rtk_pose_, msg.rtk_pose);
   se3ToMsgPose(nxt_kf_ptr->fst_opti_pose_, msg.fst_optimization_pose);
   se3ToMsgPose(nxt_kf_ptr->scd_opti_pose_, msg.scd_optimization_pose);
   frame_pub_->publish(msg);
}

void MappingNode::stop_and_save() {
    if (frontend_ptr_) {
        frontend_ptr_->requestStopAndSave();
    }
}

void MappingNode::on_timer(){
    if (!frontend_ptr_) {
        return;
    }
    publish_frame();
}


//helper functions
void MappingNode::writeVisualizationConfig(
    const std::string& config_path,
    MapMode mode)
{
    YAML::Node cfg;

    try {
        cfg = YAML::LoadFile(config_path);
    }
    catch (...) {
        cfg = YAML::Node();
    }

    auto makeColorNode = [](const std::array<float, 3>& c) {
        YAML::Node node;
        node.push_back(c[0]);
        node.push_back(c[1]);
        node.push_back(c[2]);
        return node;
    };

    auto setLayer =
        [&](YAML::Node layer,
            bool draw_map,
            bool draw_route,
            const std::string& map_pose,
            const std::string& route_pose,
            const std::array<float, 3>& map_color,
            const std::array<float, 3>& route_color)
    {
        layer["draw_map"] = draw_map;
        layer["draw_route"] = draw_route;
        layer["map_pose"] = map_pose;
        layer["route_pose"] = route_pose;
        layer["map_color"] = makeColorNode(map_color);
        layer["route_color"] = makeColorNode(route_color);
    };

    if (mode == MapMode::frontend) {
        setLayer(cfg["vis"]["layer1"], true, true,
                 "lidar", "lidar",
                 {1, 1, 1}, {1, 0, 0});

        setLayer(cfg["vis"]["layer2"], false, true,
                 "lidar", "rtk",
                 {0, 1, 0}, {0, 0, 1});
    }
    else if (mode == MapMode::replay_frontend) {
        setLayer(cfg["vis"]["layer1"], true, true,
                 "lidar_neu", "lidar_neu",
                 {1, 1, 1}, {1, 0, 0});

        setLayer(cfg["vis"]["layer2"], false, true,
                 "lidar", "rtk",
                 {0, 1, 0}, {0, 1, 0});
    }
    else if (mode == MapMode::replay_optimization) {
        setLayer(cfg["vis"]["layer1"], false, true,
                 "rtk", "rtk",
                 {1, 1, 1}, {1, 0, 0});

        setLayer(cfg["vis"]["layer2"], true, true,
                 "fst_optimization", "fst_optimization",
                 {1, 1, 1}, {0, 1, 0});
    }
    else {
        RCLCPP_ERROR(
            get_logger(),
            "Unknown MapMode: %d",
            static_cast<int>(mode));
        return;
    }

    std::ofstream fout(
        config_path,
        std::ios::out | std::ios::trunc);

    if (!fout.is_open()) {
        RCLCPP_ERROR(
            get_logger(),
            "Cannot open %s",
            config_path.c_str());
        return;
    }

    fout << cfg;
    fout.close();

    RCLCPP_INFO(
        get_logger(),
        "Wrote visualization config: %s",
        config_path.c_str());
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);   // Ctrl+C
    std::signal(SIGTERM, signalHandler);  // kill PID
    std::signal(SIGTSTP, signalHandler);  // Ctrl+Z
    auto node = std::make_shared<MappingNode>();

    rclcpp::spin(node);

    std::cout << "[main] spin exited, saving KFs...\n";

    node->stop_and_save();

    node.reset();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    std::cout << "map node shut down!\n";
    return 0;
}