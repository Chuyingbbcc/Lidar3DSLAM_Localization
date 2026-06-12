//
// Created by chuchu on 6/1/26.
//
#include <string>
#include <map>
#include <memory>
#include "Align.h"
#include "Submap.h"
#include "Optimization.h"
#include "../DataType.h"

struct BackendConfig {
    int submap_size_ = 30;
    std::string in_cloud_dir_;
    std::string in_kf_info_path_;
    IncNDTOptions inc_ndt_options_;
};

class KeyFrame;
class Backend {
public:
    Backend(const std::string& init_path);
    ~Backend() = default;
    void neuAlign();
    void buildSubmaps();
    void addKeyFrame(std::shared_ptr<KeyFrame> kf);
    void runLevel1Optimization();
    void runLevel2Optimization();
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

    void optimizeOneSubmapLevel10(Submap& submap);
    void applyLevel20Correction();
    void buildSubmapGraphNodes(std::vector<PoseGraphOptimizer::Node>& node_vect);

    bool computeSubmapRtk(const Submap& sm, Vec3d& gps_pos) const;


};


