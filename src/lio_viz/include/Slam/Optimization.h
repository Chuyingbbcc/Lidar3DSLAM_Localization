//
// Created by chuchu on 4/18/26.
//
#pragma once
#include "DataType.h"
#include <vector>
#include <functional>
#include <memory>
#include <map>
#include <stdexcept>

inline SE3d normalizedOptimizerPose(const Eigen::Isometry3d& estimate) {
    const Mat3d rotation = estimate.linear();
    if (!estimate.matrix().allFinite() ||
        !(rotation.transpose() * rotation).isApprox(Mat3d::Identity(), 1e-6) ||
        std::abs(rotation.determinant() - 1.0) > 1e-6) {
        throw std::runtime_error("Optimizer returned an invalid pose");
    }
    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();
    return SE3d(quaternion, estimate.translation());
}



enum class  OptimizationStage {
   KF_RTK_OPTI =0,
   SUBMAP_NDT,
   LOOP_CLOSURE,
   GNSS_XY_PLANAR
};
namespace g2o {
    class SparseOptimizer;
}

class PoseGraphOptimizer {
public:
    struct Node {
        size_t id_;
        double t_;
        SE3d pose_init_;
        bool has_gps_ = true;
        Vec3d gps_pos_ = Vec3d::Zero();
    };
    struct Edge {
       size_t id_i_;
       size_t id_j_;
       SE3d T_i_j_;
       Mat6d info_;
       bool is_loop_ = false;
    };
    PoseGraphOptimizer(OptimizationStage stage = OptimizationStage::KF_RTK_OPTI);
    ~PoseGraphOptimizer();
    void setNodes(const std::vector<Node>& nodes);
    void setEdges(const std::vector<Edge>& edges);
    void setGpsInfo(Mat3d& gps_info);
    void setGpsHuberDelta(double delta) {
        if(!std::isfinite(delta) || delta<=0)throw std::invalid_argument("Invalid GNSS Huber delta");
        gps_huber_delta_=delta;
    }
    bool optimize(int iterations = 20);
    void setStopRequested(std::function<bool()> stop) { stop_requested_ = std::move(stop); }
    void getOptimizedPoses(std::map<size_t,SE3d >& poses_map);
private:
    std::function<bool()> stop_requested_;
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::vector<SE3d> optimized_poses_;
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    OptimizationStage stage_;

    Mat3d gps_info_ = Mat3d::Zero();
    double gps_huber_delta_=2;

    bool buildOptimizer();
    void addVertices();
    void addRelativeEdges();
    void addOdomEdges();
    void addGpsEdges();

};



