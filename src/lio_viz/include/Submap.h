//
// Created by chuchu on 6/2/26.
//

#pragma once
#include "DataType.h"
#include <vector>
#include <memory>
#include  "KeyFrame.h"

struct Submap {
   size_t id_= 0;

   size_t start_kf_id_ =0;
   size_t end_kf_id_ =0;

   SE3d pose_init_;
   SE3d pose_optimized_;

   bool closed_ =false;
   std::vector<std::shared_ptr<KeyFrame>> key_frames_;
};

class SubmapManager {
public:
   explicit  SubmapManager(size_t submap_size =20): submap_size_(submap_size){}
   void  addKfToSubmap (const std::shared_ptr<KeyFrame>& kf);
   std::vector<Submap>& getSubmaps();
private:
   void startNewSubmap(const std::shared_ptr<KeyFrame>& kf);
   bool shouldCloseSubmap (const Submap& sm, const std::shared_ptr<KeyFrame> kf) const;
   void closeCurrentSubmap();
   size_t max_kf_size_= 10;
   size_t submap_size_ =0;
   size_t cur_submap_id_ =-1;
   std::vector<Submap> submaps_;
};



