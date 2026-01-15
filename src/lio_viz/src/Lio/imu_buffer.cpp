//
// Created by chuchu on 1/8/26.
//

#include "Lio/imu_buffer.h"
#include <algorithm>
#include <chrono>
#include <mutex>

void ImuBuffer::push(const ImuData& data) {
    {
        std::lock_guard<std::mutex>lk(m_);

        if(buf_.empty() || data.t >= buf_.back().t) {
            buf_.push_back(data);
        }
        else {
            //binary search and insert
            auto it = std::upper_bound(buf_.begin(), buf_.end(), data.t, [](double t, const ImuData& imu){return t <imu.t;});
            buf_.insert(it, data);
        }
    }
    cv_.notify_all();
}
bool ImuBuffer::wait_until(double t, double timeout_sec ) {
   std::unique_lock<std::mutex>lk(m_);

   if(timeout_sec <0) {
      cv_.wait(lk, [&]{return !buf_.empty() &&  buf_.back().t >= t;});
      return true;
   }
    return cv_.wait_for(
    lk,
    std::chrono::duration<double>(timeout_sec),
    [&] {
      return !buf_.empty() && buf_.back().t >= t;
    });
}
void ImuBuffer::extract(std::vector<ImuData>& out, const double t0, const double t1 ) {
   std::lock_guard<std::mutex>lk(m_);
   for (const auto& imu : buf_) {
      if (imu.t < t0) continue;
      if (imu.t > t1) break;
      out.push_back(imu);
   }
}
void ImuBuffer::pop_segment(std::vector<ImuData>& out, double t0, double t1, double keep_prev) {
    std::lock_guard<std::mutex> lk(m_);

    // collect IMU in [t0, t1]
    for (const auto& imu : buf_) {
        if (imu.t < t0) continue;
        if (imu.t > t1) break;
        out.push_back(imu);
    }
    // drop IMU older than (t0 - keep_before_t0)
    const double drop_t = t0 - keep_prev;
    while (!buf_.empty() && buf_.front().t < drop_t) {
        buf_.pop_front();
    }
}

void ImuBuffer::dropOlderThan(double t) {
    std::lock_guard<std::mutex> lk(m_);
    while (!buf_.empty() && buf_.front().t < t) {
        buf_.pop_front();
    }
}

bool ImuBuffer::empty() const {
    std::lock_guard<std::mutex> lk(m_);
    return buf_.empty();
}

double ImuBuffer::latest_time() const {
    std::lock_guard<std::mutex> lk(m_);
    return buf_.empty() ? -1.0 : buf_.back().t;
}