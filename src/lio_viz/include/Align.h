//
// Created by chuchu on 12/24/25.
//

#ifndef ALIGN_H
#define ALIGN_H
#include <memory>
#include <string>
#include "point_cloud_utils.h"
#include "DataType.h"
#include "common.h"
#include  <map>
#include  <list>


class RegistrationBase {
    public:
    virtual ~RegistrationBase() = default;

    virtual void AddCloud(std::shared_ptr<const PointCloud> target){};
    virtual void SetSourceCloud(std::shared_ptr<const PointCloud> source) = 0;
    virtual bool Align(SE3f& init_pose);
};

enum class NearbyType {
    CENTER,
    NEARBY6,
 };

struct IncNDTOptions {
   int max_iterations_ = 10;
   float voxel_size_ = 1.0;
   float inv_voxel_size_ = 1.0;
   int min_effective_pts_ = 10;
   int min_pts_in_voxel_ = 5;
   int max_pts_in_voxel_ = 50;

   float eps_ = 1e-3;
   float res_outlier_threshold_ = 5.0;

   size_t capacity_ = 100000;
   NearbyType nearby_type_ = NearbyType::CENTER;

};

class IncNDT final: public RegistrationBase {
public:
    IncNDT();
    IncNDT(const IncNDTOptions& opts);

    bool Align(SE3f& init_pose) override;
    void SetSourceCloud(std::shared_ptr<const PointCloud> source) override;
    void AddCloud(std::shared_ptr<const PointCloud> target) override;

  private:
    bool first_scan_processed = true;
    IncNDTOptions opt_;
    std::shared_ptr<const PointCloud> source_ptr_;

    //nearby keys
    std::vector<KeyType>nearby_grids_;

    //LRU(List + unordered_map)
    using KeyDataPair = std::pair<KeyType, VoxelData>;
    std::list<KeyDataPair>cache_;
    std::unordered_map<KeyType, std::list<KeyDataPair>::iterator, hash_vec<3>>pair_map_;

    void GenerateNearbyGrids();
    void UpdateVoxel(VoxelData& voxel_data);



};

#endif //ALIGN_H


