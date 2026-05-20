//
// Created by chuchu on 4/18/26.
//
#include "Slam/Optimization.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "DataType.h"
#include "memory.h"



class EdgeSE3GPS: public g2o::BaseUnaryEdge<3, Vec3d, g2o::VertexSE3> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeError() override {
        const auto* v = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        const SE3d& T = SE3d(v->estimate().matrix());
        Vec3d pred = T.translation();
        _error = pred - _measurement;
    }

    void linearizeOplus() override {
       _jacobianOplusXi.setZero();
       _jacobianOplusXi.block<3,3>(0,0) = Mat3d::Identity();
    }
    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

PoseGraphOptimizer::PoseGraphOptimizer()= default;

PoseGraphOptimizer::~PoseGraphOptimizer()=default;

void PoseGraphOptimizer::setNodes(const std::vector<Node> &nodes) {
    nodes_ = std::move(nodes);
}

bool PoseGraphOptimizer::optimize(int iterations) {
    if (nodes_.empty()) {
        return false;
    }
    if(!buildOptimizer()) {
        return false;
    }
    addVertices();
    addOdomEdges();
    addGpsEdges();

    optimizer_ ->initializeOptimization();
    int iters = optimizer_ -> optimize(iterations);
    return iters >0;
}

void PoseGraphOptimizer::getOptimizedPoses(std::map<size_t,SE3d >& poses_map){
   if(!optimizer_) {
      return;
   }
   for(size_t i=0; i<nodes_.size(); i++) {
       auto * v = dynamic_cast<g2o::VertexSE3*>(optimizer_ ->vertex(static_cast<int>(nodes_[i].id_)));
       if (!v ) {
        continue;
       }
      poses_map[nodes_[i].id_] = SE3d(v->estimate().matrix());
  }
}


bool PoseGraphOptimizer::buildOptimizer() {
    /*
    BlockSolver is responsible for:
     building this Hessian matrix
      solving this linear system
    pose → 6 DOF (SE3)
    landmark → 3 DOF (gps XYZ)
    */
    using BlockSolverType = g2o::BlockSolver< g2o::BlockSolverTraits<6, 6>>;
    /*
     *  So after g2o linearizes your problem, it gets a matrix equation, and LinearSolverCholmod is the tool used to solve that equation.
    *  CHOLMOD is a sparse linear algebra library designed for matrices that are: sparse symmetric positive definite
     */
    using LinearSolverType = g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;

    /*
    * OptimizationAlgorithmLevenberg   ← controls iteration (nonlinear)
        ↓
    *  BlockSolver                     ← builds H and b
        ↓
     * LinearSolverCholmod             ← solves HΔx = b
     */
    auto linear_solver = std::make_unique<LinearSolverType>();
    auto block_solver  = std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto algorithm     = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    optimizer_ = std::make_unique<g2o::SparseOptimizer>();
    optimizer_->setAlgorithm(algorithm);
    optimizer_->setVerbose(true);

    return true;
}

void PoseGraphOptimizer::addVertices() {
   if (optimizer_) {
     return;
   }
   //node->vertex
   for (size_t i=0; i<nodes_.size(); ++i) {
      const auto&node = nodes_[i];

      auto* v = new g2o::VertexSE3();
      v->setId(static_cast<int>(node.id_));
      v->setEstimate(Eigen::Isometry3d(node.pose_init_.matrix()));

      if (i==0) {
         v->setFixed(true);
      }
      optimizer_->addVertex(v);
   }
}

void PoseGraphOptimizer::addOdomEdges() {
    if (!optimizer_  || nodes_.size()<2) {
        return;
    }
    Mat6d info = Mat6d::Identity();
    //Todo:: tune parameters
    info.topLeftCorner<3,3>() *= 100;
    info.bottomRightCorner<3,3>() *= 400;

    for (size_t i=0; i+1 <nodes_.size(); ++i) {
        const SE3d& Ti = nodes_[i].pose_init_;
        const SE3d& Tj = nodes_[i+1].pose_init_;

        SE3d Tij = Ti.inverse() * Tj;

        auto* edge = new g2o::EdgeSE3();
        edge->setVertex(0, optimizer_->vertex(static_cast<int>(nodes_[i].id_)));
        edge->setVertex(1, optimizer_->vertex(static_cast<int>(nodes_[i+1].id_)));
        edge->setMeasurement(Eigen::Isometry3d(Tij.matrix()));
        edge->setInformation(info);

        //A RobustKernel is a function used in optimization (e.g., in g2o) to reduce the influence of outlier measurements by down-weighting large residual errors so they don’t distort the solution.
        auto* rk = new g2o::RobustKernelHuber();
        rk->setDelta(1.0);
        edge->setRobustKernel(rk);
        optimizer_->addEdge(edge);
    }
}

void PoseGraphOptimizer::addGpsEdges() {
    if (optimizer_) {
       return;
    }
    Eigen::Matrix3d gps_info = Eigen::Matrix3d::Identity()*10.0;
    for(size_t i=0; i<nodes_.size(); ++i) {
       const auto& node = nodes_[i];
       if(!node.has_gps_) {
           continue;
       }
       auto* edge = new EdgeSE3GPS();
       edge->setVertex(0, optimizer_->vertex(static_cast<int>(node.id_)));
       edge->setMeasurement(node.gps_pos_);
        edge->setInformation(gps_info);

        auto* rk = new g2o::RobustKernelHuber();
        rk->setDelta(2.0);
        edge->setRobustKernel(rk);

        optimizer_->addEdge(edge);
    }
}