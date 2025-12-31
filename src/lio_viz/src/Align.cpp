//
// Created by chuchu on 12/24/25.
//
#include "Align.h"
#include "../include/Align.h"
#include <Eigen/Core>

#include <numeric>
#include <omp.h>
#include "point_cloud_utils.h"
#include "../3rd_party/sophus/sophus/types.hpp"
#include "so3.hpp"
#include <map>
#include <set>

//#define Debug_NDT
using namespace std;

IncNDT::IncNDT() {
  opt_.inv_voxel_size_ = 1.0/ opt_.voxel_size_;
  GenerateNearbyGrids();
}

IncNDT::IncNDT(const IncNDTOptions &opts): opt_(opts) {
  opt_.inv_voxel_size_ = 1.0/ opt_.voxel_size_;
  GenerateNearbyGrids();
}
void IncNDT::SetSourceCloud(std::shared_ptr<const PointCloud> source) {
  source_ptr_ = source;
}

bool IncNDT::Align(SE3f& init_pose) {
  //initial pose
  SE3f new_pose = init_pose;

  int ptr_size = source_ptr_->size();
  vector<int>index(ptr_size);
  std::iota(index.begin(), index.end(), 0);

  int num_res_per_point= opt_.nearby_type_ == NearbyType::CENTER? 1: 7;
  int total_count = ptr_size * num_res_per_point;

  //iterate num_iterations
  for(int i=0; i<opt_.max_iterations_; i++) {
    std::vector<uint8_t>effect_pts(total_count, 0);
    std::vector<Eigen::Matrix<float, 3,6>>jacobians(total_count);
    std::vector<Vec3f>errors(total_count);
    std::vector<Mat3f>infos(total_count);

    //gauss-newton
    //iterate each points
#pragma omp parallel for schedule(static)
    for(int i=0; i<index.size(); i++) {
       auto p = ToVec3f((*source_ptr_)[i]);
       Vec3f q = new_pose * p;

       // figure it out the key of voxel
       Vec3i key = CastToInt(Vec3f(q* opt_.inv_voxel_size_));
      // iterate all nearby grids, and calculate Jacobian
      for(int j=0; j<nearby_grids_.size(); j++ ) {
        Vec3i nb_key = key+ nearby_grids_[j];
        auto it = pair_map_.find(nb_key);
        int cur_id = i * num_res_per_point +j;

        if(it!=pair_map_.end() && it->second->second.estimated_) {
          auto& v = it->second->second;
          Vec3f e = q - v.mu_;

          //check chi2 th
          double res= e.transpose() * v.info_ * e;
          if(std::isnan(res) || res > opt_.res_outlier_threshold_) {
            effect_pts[cur_id] = false;
            continue;
          }

          //residual
          Eigen::Matrix<float,3,6>J;
          J.block<3,3>(0,0) = -new_pose.so3().matrix() * SO3f::hat(q);
          J.block<3,3>(0,3) = Sophus::Matrix3f::Identity();

          jacobians[cur_id] =J;
          errors[cur_id] = e;
          infos[cur_id] = v.info_;
          effect_pts[cur_id] = true;
        }
        else {
          effect_pts[cur_id]= false;
        }
      }
    }
    //accumulate H and e
    int effective_num  =0 ;
    float total_res =0;
    Mat6f H = Mat6f::Zero();
    Vec6f err = Vec6f::Zero();

    for(int idx =0; idx < effect_pts.size(); idx++) {
      if (!effect_pts[idx]) {
        continue;
      }
      total_res += errors[idx].transpose() * infos[idx]* errors[idx];
      effective_num++;
      // (6*3)*(3*3)*(3*6) -> (6*6)
      H += jacobians[idx].transpose() * infos[idx]* jacobians[idx];
      //(6*3)*(3*3)*(3*1) ->(6*1)
      err +=  -jacobians[idx].transpose()* infos[idx] *errors[idx];
    }

    if(effective_num < opt_.min_effective_pts_) {
       cout<<"effective_num is too small:"<<effective_num<<endl;
      init_pose = new_pose;
       return false;
    }
    //solve
    Vec6f dx = H.inverse() *err;

    //update
    new_pose.so3() = new_pose.so3() * SO3f::exp(dx.head<3>());
    new_pose.translation() += dx.tail<3>();

#ifdef  Debug_NDT
    cout<< "iter: "<< i << endl;
    cout<< "effective num: " << effective_num << endl;
    cout << "mean res: " << total_res/effective_num << endl;
    cout<< "---------------------------------------------"<<endl;
#    endif
    //check converge
    if(dx.norm() < opt_.eps_) {
      cout<< "converge it"<< i<< endl;
      break;
    }
  }
  init_pose = new_pose;
  return true;
}

void IncNDT::UpdateVoxel(VoxelData& voxel_data) {
  // if is the first lidar frame
  if(!first_scan_processed) {
    if(voxel_data.pts_.size() >1) {
      math::computeMeanAndCov(voxel_data.pts_, voxel_data.mu_, voxel_data.sig_);
      voxel_data.info_ = voxel_data.sig_.inverse();
    }
    else {
      voxel_data.mu_ = voxel_data.pts_[0];
      voxel_data.info_ = Mat3f::Identity() *1e2;
    }
    voxel_data.estimated_ = true;
    voxel_data.pts_.clear();
    return;
  }
  // if exceed the maximum point,skip
  if(voxel_data.pts_.size() > opt_.max_pts_in_voxel_) {
    return;
  }
  // if doesn't estimate before
  if(!voxel_data.estimated_ && voxel_data.pts_.size() > opt_.min_pts_in_voxel_) {
    math::computeMeanAndCov(voxel_data.pts_, voxel_data.mu_, voxel_data.sig_);
    voxel_data.info_ = voxel_data.sig_.inverse();
    voxel_data.estimated_ =true;
    // old size;
    voxel_data.num_pts_ = voxel_data.pts_.size();
    voxel_data.pts_.clear();
  }
  // if already estimate,
  else if (voxel_data.estimated_ && voxel_data.pts_.size() > opt_.min_pts_in_voxel_) {
    Vec3f new_mu= Vec3f::Zero();
    Mat3f new_sig = Mat3f::Zero();
    math::updateMeanAndCov(voxel_data.pts_, voxel_data.num_pts_,  voxel_data.mu_, voxel_data.sig_, new_mu, new_sig);
    voxel_data.mu_ = new_mu;
    voxel_data.sig_ = new_sig;
    voxel_data.num_pts_ += voxel_data.pts_.size();
    voxel_data.pts_.clear();

    Eigen::JacobiSVD<Mat3f> svd(voxel_data.sig_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec3f lambda = svd.singularValues();
    if (lambda[1] < lambda[0] * 1e-3) {
      lambda[1] = lambda[0] * 1e-3;
    }

    if (lambda[2] < lambda[0] * 1e-3) {
      lambda[2] = lambda[0] * 1e-3;
    }

    Mat3f inv_lambda = Vec3f(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
    voxel_data.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
  }
}

void IncNDT::AddCloud(std::shared_ptr<const PointCloud> target) {
  std::set<KeyType, less_vec<3>>active_key_set;
  //iterate each point(transformed points)
  for(auto& p : (*target)) {
    auto pt = ToVec3f(p);
    //find the grid the point belongs to
    auto key = CastToInt(Vec3f(pt * opt_.inv_voxel_size_));
    auto it = pair_map_.find(key);
    if(it == pair_map_.end()) {
      //pop out the least used pair, push it to the map and LRU
      cache_.push_front({key, {pt}});
      pair_map_.insert({key, cache_.begin()});
      if(cache_.size() > opt_.capacity_) {
        pair_map_.erase(cache_.back().first);
        cache_.pop_back();
      }
    }
    else {
      it->second->second.AddPoint(pt);
      cache_.splice(cache_.begin(), cache_, it->second);
      it->second = cache_.begin();
    }
    active_key_set.emplace(key);
  }
  //update mu and covariacne
  for(auto& key : active_key_set) {
    UpdateVoxel(pair_map_[key]->second);
  }
}

void IncNDT::GenerateNearbyGrids() {
  if (opt_.nearby_type_ == NearbyType::CENTER) {
    nearby_grids_.emplace_back(KeyType::Zero());
  }
  else if(opt_.nearby_type_== NearbyType::NEARBY6) {
    nearby_grids_ ={
      KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
      KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)
    };
  }
}
