//
// Created by chuchu on 12/30/25.
//
#pragma once


#include "point_cloud_utils.h"
#include "DataType.h"
#include "Align.h"

class LidarOdometry {
  public:
  struct LoOption {
    LoOption() {}
    double kf_distance_ = 0.2;
    double kf_angle_degree_ =30;
    IncNDTOptions inc_opt_;
  };
  LidarOdometry(){}
  LidarOdometry(const LoOption& op): opt_(op){};
  virtual ~LidarOdometry() = default;
  virtual size_t AddCloud(std::shared_ptr<PointCloud>&pointcloud, SE3d& pose,bool use_guess= false){return 0;};
  virtual void SaveSubMap(const std::shared_ptr<PointCloud>point_cloud){};
  bool IsKeyFrame(const SE3d& cur_pose);
protected:
  bool is_first_frame_ = true;
  bool save_in_submap_ = true;
  SE3d last_pose_ ;
  int cnt_frame_ =0;
  size_t key_frame_idx_ = -1;
  LoOption opt_;
  std::string out_dir_ = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp";
};

class IncNDTOdometry : public LidarOdometry {
public:
  IncNDTOdometry(const LoOption& op);
  ~IncNDTOdometry() override = default;
  size_t AddCloud(std::shared_ptr<PointCloud>&pointcloud, SE3d& pose,bool use_guess= false) override;
  void SaveSubMap(const std::shared_ptr<PointCloud>point_cloud) override;
  // Add a submap in the future
private:
  std::shared_ptr<IncNDT> inc_ndt_;
  std::vector<SE3d>estimated_vec_;
};
