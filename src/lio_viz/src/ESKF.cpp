//
// Created by chuchu on 1/23/26.
//
#include "ESKF.h"


#include "DataType.h"
#include "Imu_data.h"
#include "so3.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>

ESKF::ESKF(const ESKFOptions &opt): options_(opt){
   BuildNoise();
   ofs_.open(options_.out_pos_path_, std::ios::out | std::ios::app);
   if(!ofs_) {
      throw std::runtime_error("Failed to open pos log file");
   }
   ofs_<< std::fixed << std::setprecision(6);
}


StationaryImuResult ESKF::EstimateStationary(const std::vector<ImuData>& samples,
                                             const StationaryImuOptions& options) {
    StationaryImuResult result;
    auto reject = [&](const std::string& reason) { result.reason = reason; return result; };
    for (double value : {options.duration_sec, options.max_gap_sec, options.max_gyro_mean,
                         options.max_gyro_std, options.max_acc_std, options.max_gravity_error}) {
        if (!std::isfinite(value) || value <= 0) return reject("Initialization thresholds must be finite and positive");
    }
    if (options.min_samples < 2) return reject("Initialization needs at least two samples");
    result.samples = samples.size();
    if (samples.size() < options.min_samples) return reject("Insufficient stationary IMU samples");
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& sample = samples[i];
        if (!std::isfinite(sample.t) || !sample.acc.allFinite() || !sample.gyro.allFinite())
            return reject("Nonfinite IMU measurement in initialization window");
        if (i && (sample.t <= samples[i-1].t || sample.t - samples[i-1].t > options.max_gap_sec))
            return reject("Timestamp gap or nonmonotonic IMU initialization window");
        result.mean_acc += sample.acc;
        result.gyro_bias += sample.gyro;
    }
    result.duration_sec = samples.back().t - samples.front().t;
    if (result.duration_sec + 1e-6 < options.duration_sec) return reject("Stationary IMU window is too short");
    result.mean_acc /= samples.size();
    result.gyro_bias /= samples.size();
    double acc_variance = 0, gyro_variance = 0;
    for (const auto& sample : samples) {
        acc_variance += (sample.acc - result.mean_acc).squaredNorm();
        gyro_variance += (sample.gyro - result.gyro_bias).squaredNorm();
    }
    result.acc_std = std::sqrt(acc_variance / samples.size());
    result.gyro_std = std::sqrt(gyro_variance / samples.size());
    if (!std::isfinite(result.mean_acc.norm()) || result.mean_acc.norm() < 1e-6 ||
        std::abs(result.mean_acc.norm() - 9.8) > options.max_gravity_error)
        return reject("Acceleration magnitude inconsistent with gravity; check units and stationarity");
    if (result.gyro_bias.norm() > options.max_gyro_mean || result.gyro_std > options.max_gyro_std ||
        result.acc_std > options.max_acc_std)
        return reject("Initial IMU window is not sufficiently stationary");
    // Keep registration's initial map frame. Gravity need not be along map -Z.
    // A single pose cannot separate accelerometer bias from gravity/tilt.
    result.gravity = -9.8 * result.mean_acc.normalized();
    result.valid = true;
    result.reason = "Stationary initialization accepted";
    return result;
}

StationaryImuResult ESKF::InitializeStationary(const std::vector<ImuData>& samples) {
    const auto result = EstimateStationary(samples, options_.imu_initialization);
    if (!result.valid) return result; // Rejection leaves the current filter untouched.
    cur_t_ = samples.back().t;
    R_ = SO3d();
    p_.setZero();
    v_.setZero();
    bg_ = result.gyro_bias;
    ba_.setZero();
    g_ = result.gravity;
    dx_.setZero();
    cov_.setIdentity();
    return result;
}

void ESKF::ReanchorAfterGap(double time, const SE3d& imu_pose, const Vec3d& velocity) {
    if (!std::isfinite(time) || time < cur_t_ || !imu_pose.matrix().allFinite() || !velocity.allFinite())
        throw std::invalid_argument("Invalid LO state for IMU recovery");
    cur_t_ = time;
    R_ = imu_pose.so3();
    p_ = imu_pose.translation();
    v_ = velocity;
    dx_.setZero();
    // Keep learned bias/gravity and their covariance, but discard stale motion
    // cross-correlations. These are recovery uncertainties, not missing IMU data.
    const Eigen::Matrix<double,9,9> calibration_cov = cov_.block<9,9>(9,9);
    cov_.setZero();
    cov_.block<3,3>(0,0) = Mat3d::Identity() * .01;  // 10 cm position sigma.
    cov_.block<3,3>(3,3) = Mat3d::Identity() * .25;  // 0.5 m/s velocity sigma.
    cov_.block<3,3>(6,6) = Mat3d::Identity() * std::pow(5 * M_PI / 180, 2);
    cov_.block<9,9>(9,9) = calibration_cov;
}

bool ESKF::Predict(const ImuData &imu) {
    if (imu.t <= cur_t_) {
       return false;
    }
    double dt = imu.t - cur_t_;
    //std::cout << "dt" << dt << std::endl;
    if (dt >(5* options_.imu_dt_)){
      cur_t_= imu.t;
      return false;
    }

    //norminal state
    Vec3d new_p = p_ + v_*dt  +0.5 *(R_*(imu.acc- ba_))*dt*dt + 0.5*g_*dt*dt;
    Vec3d new_v = v_ + R_ *(imu.acc -ba_)*dt + g_*dt;
    SO3d new_R = R_* SO3d::exp((imu.gyro - bg_)* dt);

    p_= new_p;
    v_ = new_v;
    R_ = new_R;

    //error state
    Mat18d F = Mat18d::Identity();
    //p vs v
    F.template block<3,3>(0,3)= Mat3d::Identity()* dt;
    //v vs R
    F.template block<3,3>(3,6)= -R_.matrix() * SO3d::hat(imu.acc - ba_) * dt;
    // v vs ba
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;
    // v vs g
    F.template block<3, 3>(3, 15) = Mat3d::Identity() * dt;
    // R vs R
    F.template block<3, 3>(6, 6) = SO3d::exp(-(imu.gyro - bg_) * dt).matrix();
    // R vs bg
    F.template block<3, 3>(6, 9) = -Mat3d::Identity() * dt;

    //std::cout << "F" << F<< std::endl;
    //predict
    dx_ = F * dx_;
    //(18* 18)
    cov_ = F * cov_.eval() * F.transpose() + Q_;

    cur_t_ = imu.t;
    return true;
}

bool ESKF::ObserveSE3(const SE3d& pose) {
   //H matrix
   Eigen::Matrix<double, 6, 18>H = Eigen::Matrix<double,6,18>::Zero();
   H.template block<3,3>(0,0)= Mat3d::Identity();
   H.template block<3,3>(3,6)= Mat3d::Identity();

   //K
   //left:(18*18)*(18*6) -> 18*6
   //right: (6*18)*(18*18)(18*6) ->6*6
   Eigen::Matrix<double, 18, 6>K = cov_ * H.transpose() *(H* cov_* H.transpose() + lidar_noise_).inverse();
   //
   Vec6d innov = Vec6d::Zero();
   innov.template head<3>()= (pose.translation()- p_);
   innov.template tail<3>()= (R_.inverse() * pose.so3()).log();

   //update error state and cov_
   //(18*6)*(6*1) -> 18*1
   dx_  = K* innov;
   // std::cout<<"K" << K << std::endl;
   //  std::cout<<"innov" << innov << std::endl;
   //std::cout<< "dx_ " << dx_.transpose()<< std::endl;
   // (18*18 -  18*6*6*18) * (18*18)
   cov_  = (Mat18d::Identity() - K*H)* cov_;
   cov_ = 0.5 *(cov_+ cov_.transpose());
   UpdateAndReset();
   return true;
}

void ESKF::BuildNoise() {
     // Q
    double gv = options_.gyro_var_;
    double av = options_.acc_var_;
    double bgv = options_.bias_gyro_var_;
    double bav = options_.bias_acc_var_;
    //std::cout<<"acc_var:<<"<<av<<std::endl;
    //std::cout<<"gyro_var:<<"<<gv<<std::endl;
    Q_.diagonal()<<0,0,0,av,av,av,gv,gv,gv,bav,bav,bav,bgv,bgv,bgv,0,0,0;

    //V
    double pn = options_.pos_noise_;
    double hn = options_.height_noise_;
    double an = options_.ang_noise_;
    std::cout<<"lidar_pos_noise:<<"<<pn<<std::endl;
    std::cout<<"height_noise_:<<"<<hn<<std::endl;
    lidar_noise_.diagonal()<<pn, pn, hn,an, an,an;
}

void ESKF::UpdateAndReset() {
   p_ += dx_.template block<3,1>(0,0);
   v_ += dx_.template block<3,1>(3,0);
   R_ = R_ * SO3d::exp(dx_.template block<3,1>(6,0));

   bg_ += dx_.template block<3,1>(9,0);
   ba_ += dx_.template block<3,1>(12,0);


   g_ += dx_.template block<3,1>(15,0);
   // std::cout<< "g: " << g_[0]<< g_[1]<<g_[2]<<std::endl;
   // std::cout<< "ba:" << ba_[0]<<ba_[1]<<ba_[2]<<std::endl;
   // std::cout<< "bg:" << bg_[0]<<bg_[1]<<bg_[2]<<std::endl;

   Mat18d J = Mat18d::Identity();
   J.template block<3,3>(6,6) = Mat3d::Identity() -0.5*SO3d::hat(dx_.template block<3,1>(6,0));
   cov_ = (J* cov_* J.transpose()).eval();

   if (publish_pos_) {
     ofs_ << p_(0) << " "<<p_(1) <<" "<<p_(2) << "\n";
   }
    dx_.setZero();
}


ESKFState ESKF::GetNorminalState() const {
  return ESKFState(cur_t_, R_, p_, v_,bg_,ba_);
}

const SE3d ESKF::GetCurrentPose() const {
  return SE3d(R_, p_);
}
