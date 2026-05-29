//
// Created by chuchu on 5/29/26.
//

#pragma once
#include "DataType.h"

struct PickPoint {
    Vec3d world_p_;
    size_t frame_id_ = -1;
    size_t layer_id_ = -1;
    size_t p_idx_in_frame_ = -1;

};

class PointPicker {
public:
    PointPicker() =default;
    ~PointPicker() = default;
    void addPoint(const Vec3d& world_p,
                  size_t frame_id,
                  size_t point_idx_in_frame, size_t layer_id);
    void clear();
    void build();
    bool pickNearest(const Vec3d& w_p, float max_dist, PickPoint& result);
private:
    std::vector<PickPoint> pick_points_1_;
    std::vector<PickPoint> pick_points_2_;
    bool dirty_= true;
};

