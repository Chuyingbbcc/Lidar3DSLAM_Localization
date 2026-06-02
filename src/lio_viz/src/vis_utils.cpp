//
// Created by chuchu on 5/29/26.
//
#include "vis_utils.h"
#include "../include/vis_utils.h"

#include "DataType.h"


void PointPicker::addPoint(const Vec3d& world_p,
                           size_t frame_id,
                           size_t point_idx_in_frame, size_t layer_id) {
    PickPoint pp;
    pp.world_p_ = world_p;
    pp.layer_id_ = layer_id;
    pp.frame_id_ = frame_id;
    pp.p_idx_in_frame_ = point_idx_in_frame;

    if (layer_id == 0) {
        pick_points_1_.push_back(pp);
    } else if (layer_id == 1) {
        pick_points_2_.push_back(pp);
    }

    dirty_ = true;
}

void PointPicker::clear() {
    pick_points_1_.clear();
    pick_points_2_.clear();
    dirty_ = true;
}

inline void PointPicker::build() {
    //Todo:: later change  it to kd_tree
    dirty_ = false;
}

bool PointPicker::pickNearest(const Vec3d& w_p, float max_dist, PickPoint& result) {
    if (dirty_) {
        build();
    }

    float best_d2 = max_dist * max_dist;
    bool found = false;
    auto searchLayer = [&](const std::vector<PickPoint>& pts) {
        for (const auto& pp : pts) {
            float d2 = (pp.world_p_ - w_p).squaredNorm();
            if (d2 < best_d2) {
                best_d2 = d2;
                result = pp;
                found = true;
            }
        }
    };

    searchLayer(pick_points_1_);
    searchLayer(pick_points_2_);

    return found;
}

bool PointPicker::pickNearestRay(const Vec3d& ray_o, const Vec3d& ray_dir,double max_dist,PickPoint& result) {
    if (dirty_) {
        build();
    }
    double best_d2 = max_dist * max_dist;
    bool found = false;
    auto searchLayer = [&](const std::vector<PickPoint>& pts) {
        for (const auto& pp : pts) {
            Vec3d v = pp.world_p_ - ray_o;
            double t =  v.dot(ray_dir);
            if (t< 0.0) {
               continue;
            }
            //the point on ray which is cloest to the point
            Vec3d closest_point = pp.world_p_ + ray_dir * t;
            double d2 = (pp.world_p_ - closest_point).squaredNorm();

            if (d2< best_d2) {
               best_d2 = d2;
                result = pp;
                found =true;
            }
        }
    };
    searchLayer(pick_points_1_);
    searchLayer(pick_points_2_);
    return found;
}
