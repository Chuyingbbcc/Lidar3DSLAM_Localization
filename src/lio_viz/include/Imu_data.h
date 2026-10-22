//
// Created by chuchu on 1/12/26.
//

#pragma once
#include <cstdint>

#include "DataType.h"
#include <vector>
#include <iostream>
#include  <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>



struct ImuData {
      double t= 0.0;
      Vec3d acc = Vec3d(0.0, 0.0, 0.0);
      Vec3d gyro = Vec3d(0.0, 0.0, 0.0);
};

inline void toImuData(ImuData& out, const sensor_msgs::msg::Imu& msg, const double t_ns,
                      const double acc_scale = 1.0) {
    // time
    out.t = t_ns;

    //std::cout<< "imu data t: " << t_ns << std::endl;

    // Convert input acceleration to m/s^2; SI input uses a scale of 1.0.
    out.acc(0) = msg.linear_acceleration.x * acc_scale;
    out.acc(1) = msg.linear_acceleration.y * acc_scale;
    out.acc(2) = msg.linear_acceleration.z * acc_scale;

    // gyro (angular_velocity)
    out.gyro(0) = msg.angular_velocity.x;
    out.gyro(1) = msg.angular_velocity.y;
    out.gyro(2) = msg.angular_velocity.z;
}





