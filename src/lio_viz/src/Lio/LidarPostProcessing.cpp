//
// Created by chuchu on 1/15/26.
//
#include "Lio/LidarPostProcessing.h"
#include "point_cloud_utils.h"

#include <algorithm>
#include <mutex>
#include <iostream>

namespace {
void updatePeak(std::atomic<size_t>& peak,size_t depth) {
    size_t previous=peak.load();
    while(depth>previous&&!peak.compare_exchange_weak(previous,depth)){}
}
}





LidarReorderBuffer::LidarReorderBuffer(double slack_sec): slack_sec_(slack_sec) {
    running_ =true;
    commit_thread_ = std::thread(&LidarReorderBuffer::commitLoop, this);
}

LidarReorderBuffer::~LidarReorderBuffer() {
   stop();
}

void LidarReorderBuffer::onRawSeen(double t1) {
 {
   std::lock_guard<std::mutex> lk(m_wm_);
   if(t1 > watermark_t1_) watermark_t1_ = t1;
 }
 cv_heap_.notify_one();
}

void LidarReorderBuffer::pushProcessed(PointCloud &scan) {
    /*struct HeapItem {
    double t1;
    std::uint64_t tie;
    PointCloud scan;
};*/
HeapItem item;
item.t1 = scan.get_time();
item.tie = tie_counter_.fetch_add(1);
item.scan= std::move(scan);
{
  std::lock_guard<std::mutex>lk(m_heap_);
  if (!running_.load() || input_closed_) {
      return;
  }
  heap_.push(std::move(item));
  updatePeak(max_heap_depth_,heap_.size());
}
cv_heap_.notify_one();
}

bool LidarReorderBuffer::waitPopOrdered(CloudPtr& out) {
   std::unique_lock<std::mutex> lk(m_ready_);
   cv_ready_.wait(lk, [&](){return !ready_.empty() || done_ || !running_.load();});
   if(ready_.empty())return false;
   out =std::move(ready_.front());
   //std::cout<<"did some pop out!"<<std::endl;
   ready_.pop_front();
   if (ready_.size() <= low_watermark_) {
       cv_backpressure_.notify_all();
   }
   return true;
}

void LidarReorderBuffer::closeInput() {
   {
      std::lock_guard<std::mutex>lk(m_heap_);
      input_closed_ = true;
   }
   cv_heap_.notify_one();
   cv_ready_.notify_one();
}

void LidarReorderBuffer::waitUntilDone() {
  std::unique_lock<std::mutex>lk(m_ready_);
  cv_ready_.wait(lk, [&] {
     std::cout<< "done"<<done_<<std::endl;
     std::cout<< ready_.size()<<std::endl;
     return !running_.load() || (done_ && ready_.empty());
  });
}

void LidarReorderBuffer::waitIfReadyTooLarge() {
    std::unique_lock<std::mutex> lk(m_ready_);

    if (ready_.size() < high_watermark_) {
        return;
    }

    cv_backpressure_.wait(lk, [&] {
        return !running_.load() || ready_.size() <= low_watermark_;
    });

}


void LidarReorderBuffer::stop() {
//if not set was_running to false, set it
bool was_running = running_.exchange(false);
if(!was_running) {
   return;
}
cv_heap_.notify_all();
cv_ready_.notify_all();
cv_backpressure_.notify_all();

if(commit_thread_.joinable()) {
   commit_thread_.join();
}
}

void LidarReorderBuffer::commitLoop() {
  while (running_.load()) {
    std::unique_lock<std::mutex>lk(m_heap_);
    cv_heap_.wait(lk, [&](){return !heap_.empty() || !running_.load() || input_closed_;});
    if (!running_.load())break;

    if(input_closed_) {
      //flush the items
      while(!heap_.empty()) {
          HeapItem item = std::move(const_cast<HeapItem&>(heap_.top()));
          heap_.pop();
          lk.unlock();
          {
             std::lock_guard<std::mutex>lk_ready(m_ready_);
             ready_.push_back(std::make_shared<PointCloud>(std::move(item.scan)));
             updatePeak(max_ready_depth_,ready_.size());
          }
          cv_ready_.notify_one();
          lk.lock();
      }
      break;
    }
    //watermark
    double wm;
    {
     std::lock_guard<std::mutex>lk_ready_(m_wm_);
     wm = watermark_t1_;
    }

    //safely pop out the ready queue
    while (!heap_.empty()) {
      const auto& top = heap_.top();
      if(wm>=0 && top.t1 <= wm-slack_sec_) {
         HeapItem item = std::move(const_cast<HeapItem&>(top));
         heap_.pop();
         //unlock from here, so it will not block producer
         lk.unlock();
         {
            std::lock_guard<std::mutex>lk_ready(m_ready_);

            ready_.push_back(std::make_shared<PointCloud>(std::move(item.scan)));
            updatePeak(max_ready_depth_,ready_.size());
         }
          cv_ready_.notify_one();
          lk.lock();
      }
      else {
        // no available item yet
        break;
      }
    }
  }
  {
     std::lock_guard<std::mutex>lk_ready(m_ready_);
     done_ =true;
  }
  cv_ready_.notify_all();
}
