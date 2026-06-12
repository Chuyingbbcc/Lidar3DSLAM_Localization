//
// Created by chuchu on 6/2/26.
//
#include <iostream>
#include "Submap.h"

void SubmapManager::addKfToSubmap(const std::shared_ptr<KeyFrame> &kf) {
   if (cur_submap_id_ == -1) {
       startNewSubmap(kf);
   }
   Submap& cs = submaps_[cur_submap_id_];
   cs.key_frames_.push_back(kf);
   cs.end_kf_id_ = kf->id_;

    if (shouldCloseSubmap(cs, kf)){
        closeCurrentSubmap();
    }
}

void SubmapManager::startNewSubmap(const std::shared_ptr<KeyFrame> &kf) {
    Submap sm;
    sm.id_ = submaps_.size();
    sm.start_kf_id_ = kf->id_;
    sm.end_kf_id_ = kf->id_;
    sm.pose_init_ = kf->lidar_pose_neu_;
    sm.pose_optimized_ = sm.pose_init_;

    submaps_.push_back(sm);
    cur_submap_id_ = sm.id_;
}

bool SubmapManager::shouldCloseSubmap(const Submap &sm, const std::shared_ptr<KeyFrame> kf) const {
    if (sm.key_frames_.size() >= max_kf_size_) {
        return true;
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
