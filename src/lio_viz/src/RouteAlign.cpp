//
// Created by chuchu on 4/23/26.
//
#include "RouteAlign.h"
#include "DataType.h"
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "../include/DataType.h"


Mat2d Rigid2D::rotationMatrix()const {
   double c = cos(yaw_);
   double s = sin(yaw_);

   Mat2d R;
   R<<c, -s,
      s, c;
   return R;
}

Vec2d Rigid2D::transform(const Vec2d& p) const {
     return rotationMatrix() * p + t_;
 }

Rigid2D RouteAlign::estimateRigid2D(const std::vector<AlignedPair> &pairs) {
  if(pairs.size() <2 ) {
     throw std::invalid_argument("Pairs must be at least two");
  }
  Vec2d mean_odom = Vec2d::Zero();
  Vec2d mean_rtk = Vec2d::Zero();

  //calculate mean
  for(const auto& p:  pairs) {
    mean_odom += p.p_odom_.head<2>();
    mean_rtk += p.p_rtk_.head<2>();
  }
  mean_odom /= pairs.size();
  mean_rtk /= pairs.size();

  //resolve the transform
  Mat2d H = Mat2d::Zero();
  for (const auto& p:  pairs) {
     Vec2d qo = p.p_odom_.head<2>()- mean_odom;
     Vec2d qr = p.p_rtk_.head<2>()- mean_rtk;
     H += qo * qr.transpose();
  }
  Eigen::JacobiSVD<Mat2d> svd(H, Eigen::ComputeFullU|Eigen::ComputeFullV);
  Mat2d U = svd.matrixU();
  Mat2d V = svd.matrixV();
  /*
    *U describes the main directions of the odom-side centered data.

  V describes the main directions of the RTK-side centered data.
   */
   Mat2d R = V* U.transpose();

    if (R.determinant() < 0.0) {
        V.col(1) *= -1.0;
        R = V * U.transpose();
    }

    Rigid2D res;
    /*
     * R=[cosθ  sinθ
     * −sinθ cosθ]
     */
    res.yaw_ = std::atan2(R(1,0), R(0,0));
    res.t_ = mean_rtk -R* mean_odom;

    return res;
}

Rigid2D RouteAlign::estimateRobustRigid2D(const std::vector<AlignedPair> &pairs, double min_motion_dist,
                                                 double keep_ratio, int iterations) {
    std::vector<AlignedPair> filtered;
    filterByMotion(pairs, filtered, 0.2);
    if (filtered.size() < 2) {
       throw std::runtime_error("Not enough pairs for robust estimation");
    }
    Rigid2D R = estimateRigid2D(filtered);
    for (int i=0; i< iterations; i++) {
        std::vector<AlignedPair> scnd_filtered;
        rejectWorstResiduals(
            filtered,
            R,
            scnd_filtered,
            keep_ratio);
        if (scnd_filtered.size() < 2) break;
        R = estimateRigid2D(scnd_filtered);
        filtered = std::move(scnd_filtered);
    }
    return R;
}

void RouteAlign::filterByMotion(const std::vector<AlignedPair> &pairs, std::vector<AlignedPair>&out,  double min_dist) {
   if(pairs.size() == 0) return;
   out.push_back(pairs.front());
   Vec3d last = pairs.front().p_odom_;
   for(size_t i = 0; i < pairs.size(); ++i) {
       if((pairs[i].p_odom_ -last).norm() >= min_dist) {
           out.push_back(pairs[i]);
           last = pairs[i].p_odom_;
       }
   }
   return;
}

void RouteAlign::compute2DResiduals(const std::vector<AlignedPair> &pairs,
    const Rigid2D &transform, std::vector<ResidualItem>& resVec) {
    resVec.clear();
    for(size_t i = 0; i < pairs.size(); ++i) {
       Vec2d po = pairs[i].p_odom_.head<2>();
       Vec2d qr = pairs[i].p_rtk_.head<2>();

       Vec2d pred = transform.transform(po);
       double r = (qr- pred).norm();
       resVec.push_back({i, r});
    }
    return;
}

void RouteAlign::rejectWorstResiduals(const std::vector<AlignedPair> &pairs,
    const Rigid2D &transform, std::vector<AlignedPair>& out,  double keep_ratio) {
    std::vector<ResidualItem> resVec;
    compute2DResiduals(pairs, transform,resVec);

    std::sort(resVec.begin(), resVec.end(), [](const ResidualItem& a, const ResidualItem& b) {
        return a.residual_ < b.residual_;
    });

    int keep_n = keep_ratio* resVec.size();
    keep_n = std::max(keep_n, 2);

    std::vector<AlignedPair>inliers;
    inliers.reserve(keep_n);

    for(size_t i = 0; i < keep_n; ++i) {
       inliers.push_back(pairs[resVec[i].index_]);
    }
    out = std::move(inliers);
    return;
}

Mat3d RouteAlign::yawToRotation3D(double yaw) {
  double c = std::cos(yaw);
  double s = std::sin(yaw);

  Mat3d R = Mat3d::Identity();
  R(0,0) = c;
  R(0,1) = -s;
  R(1,0) = s;
  R(1,1) =c;
  return R;
}

Vec3d RouteAlign::applyTo3dCoord(
    const Vec3d& p_odom,
    const Rigid2D& transform,
    double z_offset) {
    Vec3d out = Vec3d(0,0,0);
    Vec2d xy = transform.rotationMatrix() * p_odom.head<2>() + transform.t_;
    out.x() =xy.x();
    out.y() =xy.y();
    out.z() = p_odom.z() + z_offset;
    return out;
}

double RouteAlign::estimateMedianZOffset(const std::vector<AlignedPair> &pairs, const Rigid2D &transform) {
 std::vector<double> dzs;
 dzs.reserve(pairs.size());

for (const auto& p : pairs) {
    Vec3d aligned =
        applyTo3dCoord(p.p_odom_, transform, 0.0);

    dzs.push_back(p.p_rtk_.z() - aligned.z());
}
std::sort(dzs.begin(), dzs.end());
return dzs[dzs.size()/2];
}
