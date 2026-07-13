//
// Created by chuchu on 6/1/26.
//
#pragma once

#include <functional>
#include <string>
#include <map>
#include <memory>
#include "Align.h"
#include "Submap.h"
#include "Optimization.h"
#include "../DataType.h"
#include "../Submap.h"


struct LoopClosureOptions {
   int loop_min_gap_ = 10;
   double min_overlap_ = 0.7;
   double xy_thresh_ = 10.0;
   double z_thresh_ = 8.0;
   double yaw_thresh_ = 30.0 * M_PI/180.0;
   //kf loop
   int submap_radius_ = 1;
   int loop_min_kf_gap_ =50;
   double kf_xy_thresh_ = 10.0;
   double kf_z_thresh_ = 10.0;
   int min_exist_idx_gap_ = 50;
   int min_event_idx_gap_ = 20;
    //ndt eval
   double score_gate_ = 3.3;
   int valid_count_gate_ = 500;
   double valid_ratio_gate_ = 0.85;
   //
   int half_sw_size_ =10;

};
struct BackendConfig {
    int submap_size_ = 30;
    std::string in_cloud_dir_;
    std::string in_kf_info_path_;
    IncNDTOptions inc_ndt_options_;
    LoopClosureOptions loop_closure_options_;
};

struct LoopPair {
   size_t id_ = INVALID_ID;
   size_t idx_i_ =INVALID_ID;
   size_t idx_j_ = INVALID_ID;
   size_t kf_idx_i_ =INVALID_ID;
   size_t kf_idx_j_ = INVALID_ID;

   double dist_xy_ = 0.0;
   double dist_z_ = 0.0;
   double yaw_diff_ =0.0;
   double overlap_ratio_ =0.0;
   double ndt_score_ =0.0;
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
    using loop_callback = std::function<void(int)>;
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
    void runLoopClosureLocalToGlobal(loop_callback callback);
    void runKfLoopClosureLocalToGlobal(loop_callback callback);
    void runTest(loop_callback callback);
    void runCorrectNdt();

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

    std::pair<NdtEval,SE3d>runSubmapNdt(Submap& sm1, Submap& sm2, const SE3d& init_pose);

    void optimizeOneSubmapLevel10(Submap& submap);
    void applyLevel20Correction();
    void applySubmapCorrectionToKeyframes();
    void buildSubmapGraphNodes(std::vector<PoseGraphOptimizer::Node>& node_vect);
    void buildKfGraphNodesAndEdges(std::vector<PoseGraphOptimizer::Node>& node_vect, std::vector<PoseGraphOptimizer::Edge>& edge_vect);

    bool computeSubmapRtk(const Submap& sm, Vec3d& gps_pos) const;
    void markLoopClosureKfs(const std::vector<LoopPair>& loop_pairs);
    void updateLoopToKf(const std::map<size_t, SE3d>& optimized_map);
    void clearKfsLoop();

    //loop closure
    double bboxOverlapRatioXY(
     const Submap& a,
     const Submap& b) const;
    //helpers
    double preScoreLoopPair(const LoopPair& p)const;
    bool sameLoopEvent(const LoopPair& a,
                            const LoopPair& b,
                            int radius_i,
                            int radius_j) const;
    std::vector<int>nearbySubmapIds(int sid) const;
    std::vector<LoopPair> makeSubmapPairsFromKfPair(const std::shared_ptr<KeyFrame>& kf_i,
    const std::shared_ptr<KeyFrame>& kf_j);
    void selectSparseLoopPairs(std::vector<LoopPair>&in_vect, std::vector<LoopPair>&out);
    void runLoopRegistration(const std::vector<LoopPair>&pair_vect, std::vector<LoopConstraint>& loop_edges );
    void runSubmapRegistration(std::vector<LoopConstraint>& loop_edges);
    void groupLoopEvents( const std::vector<LoopPair>& raw, std::vector<std::vector<LoopPair>>& events);
    LoopPair chooseBestInEvent(const std::vector<LoopPair>& event);

    //loop closure local glbal helper
    void findLoopPairsForKf(size_t kf_id, std::vector<LoopPair>& loop_pairs);
    LoopPair findLoopPairsSlidingWindow(size_t kf_id);

    LoopPair chooseBestLoopPair(std::vector<LoopPair>& loop_pairs);
    void addCurrentNdtEdges(const std::vector<PoseGraphOptimizer::Node>& node_vect,  std::vector<PoseGraphOptimizer::Edge>& edge_vect);
    void updateOptimizationToSubmap(const std::map<size_t, SE3d>&optimized_map, std::vector<Submap>& submap_vect);
    void removeDuplicateSubmapPairs(std::vector<LoopPair>& pairs);
    std::shared_ptr<PointCloud> buildSlidingWindowCloud(size_t kf_id);
    std::pair<SE3d, NdtEval>runSlidingWindowNdt(size_t kf_i, size_t kf_j);
    void markLoopSlidingWindow(size_t kf_i, size_t kf_j);
    double getYaw(const SE3d& pose);
    Mat3d makeYawRotation(double yaw);
    SE3d keepXYAndYawOnly(const SE3d& optimized_pose,
                     const SE3d& reference_pose);



    //might remove
    LoopPair chooseBestSubmapPairFromKfPair(const std::shared_ptr<KeyFrame>& kf_i,
    const std::shared_ptr<KeyFrame>& kf_j);
    //void findLoopPairs(std::vector<LoopPair>& loop_vect);



};

