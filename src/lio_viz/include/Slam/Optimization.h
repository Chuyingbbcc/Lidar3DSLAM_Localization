//
// Created by chuchu on 4/18/26.
//
#pragma once
#include "DataType.h"
#include <vector>
#include <memory>
#include <map>


enum class  OptimizationStage {
   KF_RTK_OPTI =0,
   SUBMAP_NDT,
   LOOP_CLOSURE
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
        Vec3d gps_pos_;
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
    bool optimize(int iterations = 20);
    void getOptimizedPoses(std::map<size_t,SE3d >& poses_map);
private:
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    std::vector<SE3d> optimized_poses_;
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    OptimizationStage stage_;

    Mat3d gps_info_;

    bool buildOptimizer();
    void addVertices();
    void addRelativeEdges();
    void addOdomEdges();
    void addGpsEdges();

};



