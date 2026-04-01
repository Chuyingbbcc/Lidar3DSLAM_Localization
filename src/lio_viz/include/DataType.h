//
// Created by chuchu on 12/24/25.
//
#include <Eigen/Core>
#include <vector>
#include <so3.hpp>
#include <se3.hpp>

#include "../3rd_party/sophus/sophus/common.hpp"


#ifndef EIGENTYPE_H
#define EIGENTYPE_H

using namespace Eigen;

template<typename T>using Vec3 = Eigen::Matrix<T, 3,1>;
template<typename T>using Vec6 = Eigen::Matrix<T, 6, 1>;
template<typename T>using Vec18= Eigen::Matrix<T, 18,1>;
template <typename T> using Mat3 = Eigen::Matrix<T, 3, 3>;
template <typename T> using Mat6 = Eigen::Matrix<T, 6, 6>;
template <typename T> using Mat18 = Eigen::Matrix<T, 18,18>;

using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;
using Vec3i = Vec3<int>;
using Vec6f = Vec6<float>;
using Vec6d = Vec6<double>;
using Vec18d = Vec18<double>;
using Mat6f = Mat6<float>;
using Mat6d = Mat6<double>;
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;
using Mat18d = Mat18<double>;
using KeyType = Eigen::Matrix<int, 3,1>;
using MotionNoiseD = Eigen::Matrix<double, 18,18>;
using LidarNoiseD = Eigen::Matrix<double, 6,6>;



template <typename S, int n>
inline Eigen::Matrix<int, n ,1>CastToInt(const  Eigen::Matrix<S,n,1>& value) {
   return value.array().template round().template cast<int>();
}

//------------------------Sophous----------------------------//

using SE3f = Sophus::SE3f;
using SE3d = Sophus::SE3d;
using SO3f = Sophus::SO3f;
using SO3d = Sophus::SO3d;



#endif //EIGENTYPE_H
