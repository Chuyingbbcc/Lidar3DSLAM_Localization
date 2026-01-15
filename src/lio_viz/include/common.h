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
#include "point_cloud_utils.h"


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
   //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>> pts_ ={};
   std::vector<Vec3f>pts_={};
   Vec3f mu_ = Vec3f::Zero();
   Mat3f sig_ = Mat3f::Zero();
   Mat3f info_ = Mat3f::Zero();
   bool estimated_ = false;
   int num_pts_ = 0;

   VoxelData(){};
   VoxelData(const Vec3f& pt) {
      pts_.emplace_back(pt);
      num_pts_++;
   }
   void AddPoint(const Vec3f& pt) {
      pts_.emplace_back(pt);
       if(!estimated_) {
           num_pts_++;
       }
   }
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

template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

template<>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3,1>& v1, const Eigen::Matrix<int, 3,1>& v2)const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

inline Vec3f ToVec3f(const  PointXYZIT& pt) {
    Vec3f v;
    v(0) = pt.x;
    v(1) = pt.y;
    v(2) = pt.z;
    return v;
}

inline void transform(std::shared_ptr<PointCloud> pts, SE3f& t) {
    int n = pts->size();
    for(int i=0 ;i<n; i++) {
        auto& ptr = (*pts)[i];
        Vec3f p(ptr.x,ptr.y,ptr.z);
        Vec3f q= t*p;
        ptr.x = q[0];
        ptr.y = q[1];
        ptr.z = q[2];
    }
}

namespace math {
   void computeMeanAndCov(const std::vector<Vec3f>& pts, Vec3f& out_mu, Mat3f& out_sig);
   void updateMeanAndCov(const std::vector<Vec3f>&pts, const int old_size, const Vec3f& old_mu ,const Mat3f& old_sig, Vec3f& out_mu, Mat3f& out_sig);
}
#endif // COMMON_H
