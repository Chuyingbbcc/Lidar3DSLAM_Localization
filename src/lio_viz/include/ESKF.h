//
// Created by chuchu on 1/23/26.
//
#pragma once

#include "DataType.h"
#include  "Imu_data.h"
#include  <fstream>


struct ESKFState {
 ESKFState() = default;
 // from time, R, p, v, bg, ba
 //
 explicit ESKFState(double time, const SO3d& R = SO3d(), const Vec3d& t = Vec3d::Zero(), const Vec3d& v = Vec3d::Zero(),
                      const Vec3d& bg = Vec3d::Zero(), const Vec3d& ba = Vec3d::Zero())
        : t_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {}

    // from pose and vel
 ESKFState(double time, const SE3d& pose, const Vec3d& vel = Vec3d::Zero())
        : t_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

 double t_;
 SO3d R_;
 Vec3d p_;
 Vec3d v_;
 Vec3d bg_;
 Vec3d ba_;
};

struct StationaryImuOptions {
    double duration_sec = 5.0;
    size_t min_samples = 500;
    double max_gap_sec = 0.05;
    double max_gyro_mean = 0.05;       // rad/s; also limits bias accepted as stationary.
    double max_gyro_std = 0.02;        // Norm of per-axis standard deviations, rad/s.
    double max_acc_std = 0.15;         // Norm of per-axis standard deviations, m/s^2.
    double max_gravity_error = 0.5;    // |mean acceleration magnitude - 9.8|, m/s^2.
};

struct StationaryImuResult {
    bool valid = false;
    std::string reason;
    size_t samples = 0;
    double duration_sec = 0;
    Vec3d mean_acc = Vec3d::Zero(), gyro_bias = Vec3d::Zero();
    Vec3d gravity = Vec3d::Zero(); // In the initial sensor/map frame.
    double acc_std = 0, gyro_std = 0;
};

class ESKF {
public:
struct ESKFOptions {
  double imu_dt_ = 0.01;
  StationaryImuOptions imu_initialization;
  //Q
  double gyro_var_ = 1e-5;
  double acc_var_ = 1e-2;
  double bias_gyro_var_ = 1e-6;
  double bias_acc_var_ = 1e-4;

  //V
  double pos_noise_ = 0.1;
  double height_noise_ = 0.1;
  double ang_noise_ = 1.0* (M_PI/180.0);
  std::string out_pos_path_ = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/garden_output/route.txt";
};

ESKF(const ESKFOptions& opt);
static StationaryImuResult EstimateStationary(const std::vector<ImuData>& samples,
                                               const StationaryImuOptions& options);
StationaryImuResult InitializeStationary(const std::vector<ImuData>& samples);
Vec3d GetGravity() const { return g_; }
Vec18d GetCovarianceDiagonal() const { return cov_.diagonal(); }
void ReanchorAfterGap(double time, const SE3d& imu_pose, const Vec3d& velocity);
bool Predict(const ImuData& imu);
bool ObserveSE3(const SE3d& pose);
ESKFState GetNorminalState() const;
const SE3d GetCurrentPose() const;

private:
  double cur_t_ = 0.0;
  // error state
  Vec18d dx_ = Vec18d::Zero();
  //nominal state
  Vec3d p_ =Vec3d::Zero();
  Vec3d v_ =Vec3d::Zero();
  SO3d R_;
  Vec3d bg_ = Vec3d::Zero();
  Vec3d ba_ = Vec3d::Zero();
  Vec3d g_{0,0, -9.8};

  //cov
  Mat18d cov_ = Mat18d::Identity();
  bool fist_set = false;

  MotionNoiseD Q_ = MotionNoiseD::Zero();
  LidarNoiseD lidar_noise_ = LidarNoiseD::Zero();
  ESKFOptions options_;
  bool publish_pos_ = true;
  std::ofstream ofs_;
  void BuildNoise();
  void UpdateAndReset();

};

