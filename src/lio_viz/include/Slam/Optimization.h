//
// Created by chuchu on 4/18/26.
//
#pragma once
#include "DataType.h"
#include <vector>
#include <memory>
#include <map>

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
    PoseGraphOptimizer();
    ~PoseGraphOptimizer();
    void setNodes(const std::vector<Node>& nodes);
    bool optimize(int iterations = 20);
    void getOptimizedPoses(std::map<size_t,SE3d >& poses_map);
private:
    std::vector<Node> nodes_;
    std::vector<SE3d> optimized_poses_;
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;

    bool buildOptimizer();
    void addVertices();
    void addOdomEdges();
    void addGpsEdges();
};



