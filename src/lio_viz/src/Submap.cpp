//
// Created by chuchu on 6/2/26.
//
#include <iostream>
#include "Submap.h"

SubmapManager::SubmapManager(
    size_t submap_size,
    size_t overlap_size)
    : max_kf_size_(submap_size),
      overlap_size_(overlap_size)
{
}

void SubmapManager::addKfToSubmap(const std::shared_ptr<KeyFrame> &kf) {

    if (!kf) {
        return;
    }

    if (cur_submap_id_ == INVALID_ID) {
        startNewSubmap(kf);
        return;
    }
   Submap& cs = submaps_[cur_submap_id_];
   cs.key_frames_.push_back(kf);
   cs.owned_keyframes_.push_back(kf);
   kf->submap_id_ = cs.id_;
   cs.end_kf_id_ = kf->id_;
    if (shouldCloseSubmap(cs, kf)) {
        const size_t closed_id = cs.id_;
        closeCurrentSubmap();
        startNewSubmapWithOverlap(submaps_[closed_id]);
    }
}

void SubmapManager::startNewSubmap(const std::shared_ptr<KeyFrame> &kf) {
    Submap sm;
    sm.id_ = submaps_.size();
    sm.start_kf_id_ = kf->id_;
    sm.end_kf_id_ = kf->id_;
    sm.pose_init_ = kf->lidar_pose_neu_;
    sm.pose_optimized_ = sm.pose_init_;

    sm.T_w_s_init_ = kf->scd_opti_pose_;
    sm.T_w_s_opti_ = sm.T_w_s_init_;
    sm.T_w_s_ndt_ = sm.T_w_s_init_;

    sm.key_frames_.push_back(kf);
    sm.owned_keyframes_.push_back(kf);

    submaps_.push_back(sm);
    cur_submap_id_ = sm.id_;
}

void SubmapManager::startNewSubmapWithOverlap(
    const Submap& prev_submap) {
    Submap sm;

    sm.id_ = submaps_.size();

    const size_t n = prev_submap.key_frames_.size();
    const size_t overlap = std::min(overlap_size_, n);

    if (overlap > 0) {
        const size_t start_idx = n - overlap;

        for (size_t i = start_idx; i < n; ++i) {
            sm.key_frames_.push_back(prev_submap.key_frames_[i]);
        }
    }

    if (!sm.key_frames_.empty()) {
        auto first_kf = sm.key_frames_.front();
        auto last_kf = sm.key_frames_.back();

        sm.start_kf_id_ = first_kf->id_;
        sm.end_kf_id_ = last_kf->id_;

        sm.pose_init_ = first_kf->lidar_pose_neu_;
        sm.pose_optimized_ = sm.pose_init_;


        sm.T_w_s_init_ = first_kf->scd_opti_pose_;
        sm.T_w_s_opti_ = sm.T_w_s_init_;
        sm.T_w_s_ndt_ = sm.T_w_s_init_;
    }

    // Important:
    // overlapped KFs are only in key_frames_.
    // They are NOT in owned_keyframes_.
    // Their kf->submap_id_ remains the previous owner.

    submaps_.push_back(sm);

    cur_submap_id_ = sm.id_;
}

bool SubmapManager::shouldCloseSubmap(const Submap &sm, const std::shared_ptr<KeyFrame> kf) const {
    if (sm.key_frames_.size() >= max_kf_size_) {
        return true;
    }
    if (sm.key_frames_.empty()) {
        return false;
    }
    Vec3d start_t =
        sm.key_frames_.front()->fst_opti_pose_.translation();

    Vec3d curr_t =
        kf->fst_opti_pose_.translation();

    double dist =
        (curr_t.head<2>() - start_t.head<2>()).norm();
    //Todo::put it to configure
    return dist > 5.0;
}
void SubmapManager::closeCurrentSubmap() {
    if (cur_submap_id_ == -1) {
       std::cout<<"There is no active submap"<<std::endl;
    }
    Submap& cs = submaps_[cur_submap_id_];
    cs.closed_ = true;
    cur_submap_id_ = -1;
}


std::vector<Submap>&  SubmapManager::getSubmaps() {
    return submaps_;
}
