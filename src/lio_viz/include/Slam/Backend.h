//
// Created by chuchu on 6/1/26.
//
#pragma once

#include <string>
#include <map>
#include <memory>
#include "Align.h"
#include "Submap.h"
#include "Optimization.h"
#include "../DataType.h"


struct LoopClosureOptions {
   int loop_min_gap_ = 10;
   double min_overlap_ = 0.1;
   double xy_thresh_ = 20.0;
   double z_thresh_ = 8.0;
   double yaw_thresh_ = 30.0 * M_PI/180.0;
   //kf loop
   int loop_min_kf_gap_ =100;
   double kf_xy_thresh_ = 5.0;
   double kf_z_thresh_ = 2.0;
   int min_exist_idx_gap_ = 50;
};
struct BackendConfig {
    int submap_size_ = 30;
    std::string in_cloud_dir_;
    std::string in_kf_info_path_;
    IncNDTOptions inc_ndt_options_;
    LoopClosureOptions loop_closure_options_;
};

struct LoopPair {
   size_t id_ = -1;
   size_t idx_i_ =-1;
   size_t idx_j_ = -1;
   size_t kf_idx_i_ =-1;
   size_t kf_idx_j_ = -1;

   double dist_xy_ = 0.0;
   double dist_z_ = 0.0;
   double yaw_diff_ =0.0;
   double overlap_ratio_ =0.0;
   SE3d T_i_j_init_;
};

struct LoopConstraint {
    int idx_i_ = -1;
    int idx_j_ = -1;

    SE3d T_i_j_;        // measurement from i to j, be consistent with graph edge
    Mat6d info_ = Mat6d::Identity();

    double score_ = 0.0;
    double trans_delta_ = 0.0;
    double yaw_delta_deg_ = 0.0;

    bool valid_ = false;
};

class KeyFrame;
class Backend {
public:
    Backend(const std::string& init_path);
    ~Backend() = default;
    void neuAlign();
    void buildSubmaps();
    void addKeyFrame(std::shared_ptr<KeyFrame> kf);
    void findLoopPairs(std::vector<LoopPair>& loop_vect);
    void findLoopPairsViaKfs(std::vector<LoopPair>& loop_vect);
    void runSubmapInsideOptimization();
    void runKfRtkOptimization();
    void runLoopClosure();

    std::map<size_t, std::shared_ptr<KeyFrame>> &getKeyFrames() {
        return key_frames_;
    }

private:
    std::unique_ptr<SubmapManager> submap_manager_ptr_;
    std::map<size_t, std::shared_ptr<KeyFrame>> key_frames_;

    //backend config
    BackendConfig backend_config_;

    bool loadBackendConfig(const std::string& init_path);
    void processNewKeyFrame(std::shared_ptr<KeyFrame> kf);
    void buildSubmapPointCloud();

    SE3d runSubmapNdt(Submap& sm1, Submap& sm2, const SE3d& init_pose);

    void optimizeOneSubmapLevel10(Submap& submap);
    void applyLevel20Correction();
    void applySubmapCorrectionToKeyframes();
    void buildSubmapGraphNodes(std::vector<PoseGraphOptimizer::Node>& node_vect);
    void buildKfGraphNodesAndEdges(std::vector<PoseGraphOptimizer::Node>& node_vect, std::vector<PoseGraphOptimizer::Edge>& edge_vect);

    bool computeSubmapRtk(const Submap& sm, Vec3d& gps_pos) const;
    void markLoopClosureKfs(const std::vector<LoopPair>& loop_pairs);
    void clearKfsLoop();

    //loop closure
    double bboxOverlapRatioXY(
     const Submap& a,
     const Submap& b) const;
    //helpers
    std::vector<int>nearbySubmapIds(int sid) const;
    LoopPair chooseBestSubmapPairFromKfPair(const std::shared_ptr<KeyFrame>& kf_i,
    const std::shared_ptr<KeyFrame>& kf_j);
    void selectSparseLoopPairs(std::vector<LoopPair>&in_vect, std::vector<LoopPair>&out);
    void runLoopRegistration(const std::vector<LoopPair>&pair_vect, std::vector<LoopConstraint>& loop_edges );
    void runSubmapRegistration(std::vector<LoopConstraint>& loop_edges);

    //void findLoopPairs(std::vector<LoopPair>& loop_vect);
};

