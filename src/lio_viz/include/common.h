// common.h
//
// Created by chuchu on 12/12/25.
//

#ifndef COMMON_H
#define COMMON_H

#include <cstddef>
#include <initializer_list>
#include <vector>
#include "DataType.h"
#include <Eigen/Core>


// Generic N-dimensional point
template<typename T, std::size_t N>
class Point {
public:
    T data[N];

    constexpr Point() : data{} {}

    Point(std::initializer_list<T> init) {
        std::size_t i = 0;
        for (T v : init) {
            if (i < N) {
                data[i++] = v;
            } else {
                break;
            }
        }
        for (; i < N; ++i) {
            data[i] = T{};
        }
    }

    constexpr T& operator[](std::size_t i) { return data[i]; }
    constexpr const T& operator[](std::size_t i) const { return data[i]; }
    constexpr std::size_t size() const { return N; }
};


struct VoxelData {
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>> pts_ ={};
   Vec3f mu_ = Vec3f::Zero();
   Mat3f sig_ = Mat3f::Zero();
   Mat3f info_ = Mat3f::Zero();
   bool estimated_ = false;
   int num_pts_ = 0;
};

template<int N>
struct hash_vec {
   size_t operator()(const Eigen::Matrix<int, N,1>& v) const {
       size_t seed =0;
       for(int i=0; i<N ; ++i) {
          seed ^= std::hash<int>()(v[i])+0x9e3779b9 + (seed<<6) + (seed>>2);
       }
       return seed;
   }
};

namespace math {
   void computeMeanAndCov(const std::vector<Vec3f>& pts, Vec3f& out_mu, Mat3f& out_sig);
   void updateMeanAndCov(const std::vector<Vec3f>&pts, const int old_size, const Vec3f& old_mu ,const Mat3f& old_sig, Vec3f& out_mu, Mat3f& out_sig);
}
#endif // COMMON_H
