//
// Created by chuchu on 12/24/25.
//
#include <Eigen/Core>
#include <vector>
#include <so3.hpp>
#include <se3.hpp>


#ifndef EIGENTYPE_H
#define EIGENTYPE_H

using namespace Eigen;

template<typename T>using Vec3 = Eigen::Matrix<T, 3,1>;
template<typename T>using Vec6 = Eigen::Matrix<T, 6, 1>;
template <typename T> using Mat3 = Eigen::Matrix<T, 3, 3>;
template <typename T> using Mat6 = Eigen::Matrix<T, 6, 6>;


using Vec3f = Vec3<float>;
using Vec3i = Vec3<int>;
using Vec6f = Vec6<float>;
using Mat6f = Mat6<float>;
using Mat3f = Mat3<float>;
using KeyType = Eigen::Matrix<int, 3,1>;



template <typename S, int n>
inline Eigen::Matrix<int, n ,1>CastToInt(const  Eigen::Matrix<S,n,1>& value) {
   return value.array().template round().template cast<int>();
}

//------------------------Sophous----------------------------//

using SE3f = Sophus::SE3f;
using SO3f = Sophus::SO3f;


#endif //EIGENTYPE_H
