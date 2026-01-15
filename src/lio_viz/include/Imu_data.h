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
      int64_t t= 0.0;
      Vec3f acc = Vec3f(0.0, 0.0, 0.0);
      Vec3f gyro = Vec3f(0.0, 0.0, 0.0);
};

inline void toImuData(ImuData& out, const sensor_msgs::msg::Imu& msg, const int64_t t_ns) {
    // time
    out.t = t_ns;

    std::cout<< "imu data t: " << t_ns << std::endl;

    // accel (linear_acceleration)
    out.acc(0) = msg.linear_acceleration.x;
    out.acc(1) = msg.linear_acceleration.y;
    out.acc(2) = msg.linear_acceleration.z;

    // gyro (angular_velocity)
    out.gyro(0) = msg.angular_velocity.x;
    out.gyro(1) = msg.angular_velocity.y;
    out.gyro(2) = msg.angular_velocity.z;
}





