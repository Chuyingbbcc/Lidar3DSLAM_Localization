//
// Created by chuchu on 4/23/26.
//

#ifndef ROUTEALIGN_H
#define ROUTEALIGN_H

#endif //ROUTEALIGN_H

#pragma once
#include <Eigen/Core>
#include "DataType.h"
#include <vector>

struct ResidualItem {
    size_t index_ = 0;
    double residual_ = 0.0;
};

struct AlignedPair {
    double timestamp_ = 0.0;
    Vec3d p_odom_ = Vec3d::Zero();
    Vec3d p_rtk_  = Vec3d::Zero();
};

struct Rigid2D {
    double yaw_ =0.0;
    Vec2d t_ = Vec2d::Zero();
    Mat2d rotationMatrix() const;
    Vec2d transform(const Vec2d& p) const;
};

class RouteAlign {
public:
    //core estimate
    static Rigid2D estimateRigid2D(
       const std::vector<AlignedPair>& pairs);

    static Rigid2D estimateRobustRigid2D(
        const std::vector<AlignedPair>& pairs,
        double min_motion_dist = 1.0,
        double keep_ratio = 0.85,
        int iterations = 2);
    //filtering

    static void filterByMotion(
     const std::vector<AlignedPair>& pairs, std::vector<AlignedPair>&out,
     double min_dist);

    // --- residuals ---
    static void compute2DResiduals(
        const std::vector<AlignedPair>& pairs,
        const Rigid2D& transform, std::vector<ResidualItem>& resVec);

    static void rejectWorstResiduals(
        const std::vector<AlignedPair>& pairs,
        const Rigid2D& transform,
        std::vector<AlignedPair>& out,
        double keep_ratio);

    // --- helpers ---
    static Mat3d yawToRotation3D(double yaw);

    static Vec3d applyTo3dCoord(
        const Vec3d& p_odom,
        const Rigid2D& transform,
        double z_offset = 0.0);

    static double estimateMedianZOffset(
        const std::vector<AlignedPair>& pairs,
        const Rigid2D& transform);

};



