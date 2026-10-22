//
// Created by chuchu on 1/8/26.
//

#include "Lio/imu_buffer.h"
#include <algorithm>
#include <chrono>
#include <mutex>
#include <cmath>

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
bool ImuBuffer::extractCovered(std::vector<ImuData>& out, double t0, double t1, double max_gap) const {
    out.clear();
    if (!std::isfinite(t0) || !std::isfinite(t1) || !std::isfinite(max_gap) || t1 <= t0 || max_gap <= 0) return false;
    std::lock_guard<std::mutex> lock(m_);
    if (buf_.empty() || buf_.front().t > t0 || buf_.back().t < t1) return false;
    auto lower = [&](double t) {
        return std::lower_bound(buf_.begin(), buf_.end(), t,
            [](const ImuData& d, double time) { return d.t < time; });
    };
    auto first = lower(t0), last = lower(t1);
    auto before = first;
    if (first->t != t0) --before;
    for (auto it = before; it != last; ++it) {
        const auto next = std::next(it);
        if (next->t <= it->t || next->t - it->t > max_gap) return false;
    }
    auto sample = [&](double t) {
        const auto right = lower(t);
        if (right->t == t) return *right;
        const auto left = std::prev(right);
        const double f = (t - left->t) / (right->t - left->t);
        ImuData d;
        d.t = t;
        d.acc = left->acc + f * (right->acc - left->acc);
        d.gyro = left->gyro + f * (right->gyro - left->gyro);
        return d;
    };
    out.push_back(sample(t0));
    for (auto it = first; it != last; ++it) {
        if (it->t > t0) out.push_back(*it);
    }
    out.push_back(sample(t1));
    return true;
}
