//
// Created by chuchu on 6/2/26.
//

#pragma once
#include "DataType.h"
#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include  "KeyFrame.h"

struct Submap {
   size_t id_= 0;

   size_t start_kf_id_ =0;
   size_t end_kf_id_ =0;

   SE3d pose_init_;
   SE3d pose_optimized_;

   SE3d T_w_s_init_;
   SE3d T_w_s_ndt_;
   SE3d T_w_s_opti_;

   bool closed_= false;

   double min_x_ =  1e30;
   double min_y_ =  1e30;
   double max_x_ = -1e30;
   double max_y_ = -1e30;
   bool has_range_ = false;


   // Includes overlap KFs, used for NDT registration
   std::vector<std::shared_ptr<KeyFrame>> key_frames_;

   // Unique owner KFs, used when applying optimized result
   std::vector<std::shared_ptr<KeyFrame>> owned_keyframes_;

   std::shared_ptr<PointCloud> point_cloud_ptr_;


    const SE3d& getOptiPose() const {
        return T_w_s_opti_;
    }

    Vec3d position() const {
        return T_w_s_opti_.translation();
    }

    double distanceXY(const Submap& other) const {
        Vec3d pi = position();
        Vec3d pj = other.position();

        return (pi.head<2>() - pj.head<2>()).norm();
    }

    double distanceZ(const Submap& other) const {
        return std::abs(position().z() - other.position().z());
    }

    SE3d relativePoseTo(const Submap& other) const {
        return getOptiPose().inverse() * other.getOptiPose();
    }

    double yaw() const {
        Mat3d R = getOptiPose().rotationMatrix();
        return std::atan2(R(1, 0), R(0, 0));
    }

    double yawDiffTo(const Submap& other) const {
        double d = other.yaw() - yaw();

        while (d > M_PI) {
            d -= 2.0 * M_PI;
        }

        while (d < -M_PI) {
            d += 2.0 * M_PI;
        }

        return std::abs(d);
    }

    Vec3d centerPosition() const {
        Vec3d sum = Vec3d::Zero();
        int count = 0;

        for (const auto& kf : key_frames_) {
            if (!kf) continue;

            SE3d T = kf->scd_opti_pose_;  // or final pose before loop
            sum += T.translation();
            count++;
        }

        if (count == 0) {
            return T_w_s_opti_.translation();
        }

        return sum / static_cast<double>(count);
    }

    bool getWorldBboxXY(
    double& min_x,
    double& min_y,
    double& max_x,
    double& max_y) const
    {
        if (!point_cloud_ptr_ || !point_cloud_ptr_->hasBBox()) {
            return false;
        }

        double lx0 = point_cloud_ptr_->minX();
        double ly0 = point_cloud_ptr_->minY();
        double lx1 = point_cloud_ptr_->maxX();
        double ly1 = point_cloud_ptr_->maxY();

        std::vector<Vec3d> corners = {
            Vec3d(lx0, ly0, 0.0),
            Vec3d(lx1, ly0, 0.0),
            Vec3d(lx1, ly1, 0.0),
            Vec3d(lx0, ly1, 0.0)
        };

        min_x =  1e30;
        min_y =  1e30;
        max_x = -1e30;
        max_y = -1e30;

        for (const auto& c : corners) {
            Vec3d w = T_w_s_init_ * c;

            min_x = std::min(min_x, w.x());
            min_y = std::min(min_y, w.y());
            max_x = std::max(max_x, w.x());
            max_y = std::max(max_y, w.y());
        }

        return true;
    }
};



class SubmapManager {
public:
   explicit SubmapManager(
        size_t submap_size = 20,
        size_t overlap_size = 5);
   void  addKfToSubmap (const std::shared_ptr<KeyFrame>& kf);
   std::vector<Submap>& getSubmaps();
private:
   static constexpr size_t INVALID_ID =
        std::numeric_limits<size_t>::max();
   void startNewSubmap(const std::shared_ptr<KeyFrame>& kf);
   void startNewSubmapWithOverlap(const Submap& prev_submap);
   bool shouldCloseSubmap (const Submap& sm, const std::shared_ptr<KeyFrame> kf) const;
   void closeCurrentSubmap();
   size_t max_kf_size_= 20;
   size_t overlap_size_ = 5;
   size_t submap_size_ =0;
   size_t cur_submap_id_ =INVALID_ID;
   std::vector<Submap> submaps_;
};



