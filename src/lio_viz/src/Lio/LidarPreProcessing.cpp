//
// Created by chuchu on 1/9/26.
//

#include "Lio/LidarPreProcessing.h"
#include "point_cloud_utils.h"
#include <cstddef>
#include <mutex>
#include <memory>
#include <functional>
#include <iostream>
#include <ostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Lio/thread_pool.h>



LidarPreProcessing::LidarPreProcessing(std::size_t num_thread): thread_pool_(num_thread) {
}

void LidarPreProcessing::onLidarMsg(const sensor_msgs::msg::PointCloud2& msg,int64_t ts, const LioOptions& options) {
    {
    std::lock_guard<std::mutex>lk(m_wm_);
        if ( ts > watermark_t1) {
            watermark_t1 = ts;
        }
    }
    std::cout<<"lidar time: "<< ts << std::endl;
    const auto ops = options;
    auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);
    thread_pool_.try_enqueue([this, ts, msg_ptr, ops]() {
       PointCloud point_cloud;
       point_cloud.setTime(ts);
       pc_pre_(msg_ptr, point_cloud, ops);
      dq_.push(std::move(point_cloud));
    });
}

void LidarPreProcessing::postProcess() {
   PointCloud pc;
   while (dq_.wait_pop(pc)) {
        std::cout << "[DONE] scan ["
                  << pc.get_time()
                  << "] points=" << pc.size()
                  << "\n";
   }
}

double LidarPreProcessing::watermark() const {
    std::lock_guard<std::mutex> lk(m_wm_);
    return watermark_t1;
}


void LidarPreProcessing::stop() {
    dq_.stop();
   thread_pool_.stop();
}

void LidarPreProcessing::onPreprocess(PointCloudPre callback) {
  pc_pre_ = std::move(callback);
}