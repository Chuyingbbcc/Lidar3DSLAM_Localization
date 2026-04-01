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

class ESKF {
public:
struct ESKFOptions {
  double imu_dt_ = 0.01;
  //Q
  double gyro_var_ = 1e-5;
  double acc_var_ = 1e-2;
  double bias_gyro_var_ = 1e-6;
  double bias_acc_var_ = 1e-4;

  //V
  double pos_noise_ = 0.1;
  double height_noise_ = 0.1;
  double ang_noise_ = 1.0* (M_PI/180.0);
  std::string out_pos_path_ = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/route.txt";
};

ESKF(const ESKFOptions& opt);
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
  Mat18d cov_ = Mat18d::Zero();
  bool fist_set = false;

  MotionNoiseD Q_ = MotionNoiseD::Zero();
  LidarNoiseD lidar_noise_ = LidarNoiseD::Zero();
  ESKFOptions options_;
  bool publish_pos_ = true;
  std::ofstream ofs_;
  void BuildNoise();
  void UpdateAndReset();

};

