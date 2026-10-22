//
// Created by chuchu on 12/12/25.
//
#include "mapping_node.h"
#include "../include/mapping_node.h"


#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>
#include "TerminalStopKeys.h"
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

static volatile std::sig_atomic_t g_exit_requested = 0;
static std::atomic<bool> g_worker_stop{false};

void signalHandler(int sig) {
    g_exit_requested = sig;
}

MappingNode::MappingNode(): Node("mapping_node") {
this->declare_parameter<std::string>("mode", "mapping");
mode_ = this->get_parameter("mode").as_string();
replay_speed_ = declare_parameter<double>("replay_speed", 1.0);
if (!std::isfinite(replay_speed_) || replay_speed_ <= 0)
    throw std::invalid_argument("replay_speed must be finite and positive");
std::cout<<"Current mode is: "<<mode_<<std::endl;

pub_ = create_publisher<std_msgs::msg::String>("pcd_path", 10);
frame_pub_ = create_publisher<lio_msgs::msg::FrameData>("frame_data", 10);
const std::string init_path = declare_parameter<std::string>("config_path",
    "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/Config/config.yaml");
frontend_ptr_ = std::make_unique<Frontend>(init_path);
//run frontend

//pause/resume control
loop_check_pub_ = create_publisher<std_msgs::msg::UInt64>(
            "/loop_closure/check_result", 1);
viewer_ready_sub_ = create_subscription<std_msgs::msg::UInt64>(
    "/loop_closure/viewer_ready", 1,
    std::bind(&MappingNode::onViewerReady, this, std::placeholders::_1));
viewer_loaded_sub_ = create_subscription<std_msgs::msg::UInt64>(
    "/loop_closure/viewer_loaded", 1,
    std::bind(&MappingNode::onViewerLoaded, this, std::placeholders::_1));
loop_resume_sub_ = create_subscription<std_msgs::msg::UInt64>(
            "/loop_closure/resume",
            1, std::bind( &MappingNode::onLoopResume, this,
                std::placeholders::_1));

if (mode_  == "mapping") {
  writeVisualizationConfig(init_path, MapMode::frontend);
  slam_thread_ = std::thread([this]() {
    try {
      if (!frontend_ptr_->init() || !frontend_ptr_->Run()) rclcpp::shutdown();
    }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Frontend exception: %s", e.what());
    rclcpp::shutdown();
  }
  catch(...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown frontend exception.");
    rclcpp::shutdown();
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
  loadKeyFrames("/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/garden_output/kf_output.txt", kf_map);
  for(auto it : kf_map) {
    replay_kf_q_.push(it.second);
  }
}

else if (mode_ == "optimization") {
    writeVisualizationConfig(init_path, MapMode::optimization);
    optimization_thread_ = std::thread([this,init_path]() {
        try { runOptimizationMode(init_path); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Backend/save failure: %s", e.what());
            rclcpp::shutdown();
        }
    });
 // Backend backend = Backend(init_path);
 // backend.buildSubmaps();
 // //backend.neuAlign();
 // //update kf fst_opti_pose_
 // backend.runKfRtkOptimization();
 // //update kf scd_opti_pose_
 // //backend.runSubmapInsideOptimization();
 // //update kf loop_opti_pose_, sm T_w_s_opti_
 // backend.runLoopClosure();
 // //test loop closure
 // const std::string kf_path = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/garden_output/kf_output.txt";
 // writeKeyFramesToFile(kf_path, backend.getKeyFrames());
 // std::cout<<"finish here"<<std::endl;
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
    frontend_ptr_->requestStop();
    RCLCPP_INFO(this->get_logger(), "Frontend stopped");
  }

  if (slam_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Joining slam thread");
    slam_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Slam thread joined");
  }
  if (optimization_thread_.joinable()) {
    optimization_thread_.join();
  }

  RCLCPP_INFO(this->get_logger(), "Destructor done");
}

void MappingNode::publish_frame() {
  std::shared_ptr<KeyFrame>nxt_kf_ptr = nullptr;
  uint64_t refresh_id = 0;
  bool refresh_last = false;
  // if (mode_ == "optimization") {
  //   return;
  // }
  if (mode_ == "mapping") {
    nxt_kf_ptr = frontend_ptr_->takeNextKeyFrame();
 }
 else {
   if ((mode_ == "rfront" || mode_ == "roptimization") &&
       frame_pub_->get_subscription_count() == 0) { replay_clock_started_ = false; return; }
   std::lock_guard<std::mutex> lock(replay_mutex_);
   if (replay_enabled_ && !replay_kf_q_.empty()) {
      if (mode_ == "rfront" || mode_ == "roptimization") {
          const auto now = std::chrono::steady_clock::now();
          if (!replay_clock_started_) {
              replay_first_time_ = replay_kf_q_.front()->time_;
              replay_started_ = now;
              replay_clock_started_ = true;
          }
          const double elapsed = std::chrono::duration<double>(now-replay_started_).count();
          if (elapsed < (replay_kf_q_.front()->time_-replay_first_time_) / replay_speed_) return;
      }
      nxt_kf_ptr = replay_kf_q_.front();
      replay_kf_q_.pop();
      refresh_id = replay_refresh_id_;
      refresh_last = refresh_id != 0 && replay_kf_q_.empty();
   }
 }
 publishKeyFrameForVis(nxt_kf_ptr, refresh_id, refresh_last);
 if (refresh_last) {
   std::lock_guard<std::mutex> lock(loop_pause_mutex_);
   if (refresh_id == refresh_id_) snapshot_sent_ = true;
 }
}

void MappingNode::publishAllKeyFramesForVis(
    const std::map<size_t, std::shared_ptr<KeyFrame>>& kfs) {
  for (const auto& entry : kfs) {
    publishKeyFrameForVis(entry.second);
  }
}

void MappingNode::publishKeyFrameForVis(
    const std::shared_ptr<KeyFrame>& nxt_kf_ptr,
    uint64_t refresh_id, bool refresh_last) {
 if (!nxt_kf_ptr) {
   return;
 }
 lio_msgs::msg::FrameData msg;
 msg.refresh_id = refresh_id;
 msg.refresh_last = refresh_last;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  msg.frame_id = nxt_kf_ptr->id_;              // change to your real field
  msg.submap_id = nxt_kf_ptr->submap_id_;
  msg.loop_role = nxt_kf_ptr->loop_role_;
  //std::cout<< "submap id: "<< msg.submap_id <<std::endl;
  msg.cloud_path = nxt_kf_ptr->cloud_path_;
  msg.is_loop_closure = nxt_kf_ptr->is_loop_closure_;  // change to your real field

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
   msg.has_rtk_pose = nxt_kf_ptr->rtk_valid_;
   se3ToMsgPose(nxt_kf_ptr->fst_opti_pose_, msg.fst_optimization_pose);
   se3ToMsgPose(nxt_kf_ptr->scd_opti_pose_, msg.scd_optimization_pose);
   se3ToMsgPose(nxt_kf_ptr->loop_opti_pose_, msg.loop_optimization_pose);
   frame_pub_->publish(msg);
}

void MappingNode::stop_and_save() {
    if (frontend_ptr_ && mode_ == "mapping") {
        std::cout<< "stop mapping mode!"<<std::endl;
        frontend_ptr_->requestStop();
        if (slam_thread_.joinable()) slam_thread_.join();
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

    cfg["vis"]["show_accumulated_map"] =
        mode == MapMode::replay_frontend || mode == MapMode::replay_optimization || mode == MapMode::frontend;
    cfg["vis"]["show_current_frame"] = mode != MapMode::replay_optimization && mode != MapMode::replay_frontend;
    cfg["vis"]["show_loop_pairs"] = mode == MapMode::optimization ||
        mode == MapMode::replay_frontend || mode == MapMode::replay_optimization;
    cfg["vis"]["dim_non_loop"] = mode == MapMode::replay_frontend || mode == MapMode::replay_optimization;
    cfg["vis"]["hide_ground"] = mode != MapMode::replay_frontend && mode != MapMode::replay_optimization;

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
                 {1, 1, 1}, {0.4f, 0.4f, 0.45f});

        setLayer(cfg["vis"]["layer2"], false, false,
                 "lidar", "rtk",
                 {0, 1, 0}, {0, 1, 0});
    }
    else if (mode == MapMode::replay_optimization) {
        setLayer(cfg["vis"]["layer1"], false, false,
                 "lidar_neu", "lidar_neu",
                 {1, 1, 1}, {0.4f, 0.4f, 0.45f});

        setLayer(cfg["vis"]["layer2"], true, true,
                 "loop_optimization", "loop_optimization",
                 {1, 1, 1}, {0.4f, 0.4f, 0.45f});
    }
    else if (mode == MapMode::optimization) {
        setLayer(cfg["vis"]["layer1"], false, false,
                 "loop_optimization", "lidar_neu",
                 {1, 0, 1}, {1, 0, 0});

        setLayer(cfg["vis"]["layer2"], true, true,
                 "loop_optimization", "loop_optimization",
                 {1,1 , 1}, {0, 1, 0});
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
void MappingNode::onViewerReady(const std_msgs::msg::UInt64::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(loop_pause_mutex_);
        if (!awaiting_inspection_ || msg->data != refresh_id_) return;
        viewer_ready_ = true;
    }
    loop_pause_cv_.notify_all();
}

void MappingNode::onViewerLoaded(const std_msgs::msg::UInt64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(loop_pause_mutex_);
    if (awaiting_inspection_ && snapshot_sent_ && msg->data == refresh_id_)
        viewer_rendered_ = true;
    loop_pause_cv_.notify_all();
}

void MappingNode::onLoopResume(
    const std_msgs::msg::UInt64::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(loop_pause_mutex_);
        if (!awaiting_inspection_ || !snapshot_sent_ || msg->data != refresh_id_) return;
        loop_resume_requested_ = true;
    }

    loop_pause_cv_.notify_one();
}

void MappingNode::pauseForLoopInspection() {
    std::unique_lock<std::mutex> lock(loop_pause_mutex_);
    while (rclcpp::ok() && !(loop_resume_requested_ && viewer_rendered_)) {
        loop_pause_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    awaiting_inspection_ = false;
}

void MappingNode::refreshForLoopInspection(
    const std::map<size_t, std::shared_ptr<KeyFrame>>& kfs, size_t count) {
    // Copy only keyframe metadata/poses; point cloud buffers remain shared.
    std::queue<std::shared_ptr<KeyFrame>> snapshot;
    for (const auto& entry : kfs) {
        if (snapshot.size() >= count) break;
        if (entry.second && entry.second->scd_opti_pose_.matrix().allFinite())
            snapshot.push(std::make_shared<KeyFrame>(*entry.second));
    }
    if (snapshot.empty()) return;
    {
        std::lock_guard<std::mutex> lock(replay_mutex_);
        replay_enabled_ = false;
        std::queue<std::shared_ptr<KeyFrame>>().swap(replay_kf_q_);
    }
    uint64_t id;
    {
        std::lock_guard<std::mutex> lock(loop_pause_mutex_);
        id = ++refresh_id_;
        viewer_ready_ = false;
        viewer_rendered_ = false;
        loop_resume_requested_ = false;
        snapshot_sent_ = false;
        awaiting_inspection_ = true;
    }
    std_msgs::msg::UInt64 reset;
    reset.data = id;
    loop_check_pub_->publish(reset);
    {
        std::unique_lock<std::mutex> lock(loop_pause_mutex_);
        auto retry_at = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (rclcpp::ok() && !viewer_ready_) {
            loop_pause_cv_.wait_for(lock, std::chrono::milliseconds(100));
            if (!viewer_ready_ && rclcpp::ok() && std::chrono::steady_clock::now() >= retry_at) {
                lock.unlock();
                loop_check_pub_->publish(reset);
                lock.lock();
                retry_at = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            }
        }
        if (!rclcpp::ok()) { awaiting_inspection_ = false; return; }
    }
    {
        std::lock_guard<std::mutex> lock(replay_mutex_);
        replay_kf_q_.swap(snapshot);
        replay_refresh_id_ = id;
        replay_enabled_ = true;
    }
    pauseForLoopInspection();
}

void MappingNode::runOptimizationMode(const std::string& init_path) {
    Backend backend(init_path, [] { return g_worker_stop.load(std::memory_order_relaxed); });
    backend.buildSubmaps();
    try {
        if (!g_worker_stop.load()) backend.neuAlign();
        backend.runKfRtkOptimization();
        //backend.runCorrectNdt();
        backend.runKfLoopClosureLocalToGlobal([this, &backend](int count) {
            refreshForLoopInspection(backend.getKeyFrames(), static_cast<size_t>(count));
        });
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Backend interrupted by error: %s; saving keyframes", e.what());
    }
    if (backend.getKeyFrames().empty()) return; // Never replace input with an empty checkpoint.
    const auto config = YAML::LoadFile(init_path);
    const std::string kf_path = config["backend"]["in_kf_info_path"].as<std::string>();
    writeKeyFramesToFile(kf_path, backend.getKeyFrames());
    backend.writeEvaluationLoops(kf_path + ".loops.txt");
    RCLCPP_INFO(get_logger(), "Saved %zu backend keyframes to %s%s",
        backend.getKeyFrames().size(), kf_path.c_str(),
        g_worker_stop.load() ? " (interrupted checkpoint)" : "");
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGTSTP, signalHandler);
    TerminalStopKeys terminal_keys;
    std::shared_ptr<MappingNode> node;
    int result = 0;
    try {
        node = std::make_shared<MappingNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        while (rclcpp::ok() && !g_exit_requested) {
            if (terminal_keys.stopPressed()) { g_exit_requested = SIGINT; break; }
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& e) {
        std::cerr << "Mapping node failure: " << e.what() << std::endl;
        result = 1;
    } catch (...) {
        std::cerr << "Unknown mapping node failure" << std::endl;
        result = 1;
    }
    g_worker_stop.store(true, std::memory_order_relaxed);
    // Release backend inspection waits before joining any worker.
    rclcpp::shutdown();
    if (node) {
        try {
            node->stop_and_save();
        } catch (const std::exception& e) {
            std::cerr << "Failed saving keyframes: " << e.what() << std::endl;
            result = 1;
        }
        node.reset();
    }
    return result;
}
