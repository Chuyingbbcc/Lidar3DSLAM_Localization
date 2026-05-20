//
// Created by chuchu on 1/12/26.
//

#pragma once
#include <deque>
#include <mutex>
#include <condition_variable>

template <typename T>
class DoneQueue {
public:
   DoneQueue(std::size_t high_watermark = 100,
   std::size_t low_watermark = 60):high_watermark_(high_watermark),
   low_watermark_(low_watermark) {}

   void push(T&& item) {
     {
        std::lock_guard<std::mutex>lk(m_);
        if (stopped_) {
          return;
        }
        q_.push_back(std::move(item));
     }
      cv_not_empty_.notify_one();
   }

   bool wait_pop(T& out) {
     std::unique_lock<std::mutex> lk(m_);
     cv_not_empty_.wait(lk, [&] { return stopped_ ||!q_.empty(); });
     if (q_.empty()) return false;
     out = std::move(q_.front());
     q_.pop_front();

     if (q_.size() <= low_watermark_) {
         cv_below_watermark_.notify_all();
     }
     return true;
   }

   void wait_if_too_large () {
      std::unique_lock<std::mutex> lk(m_);
      if (q_.size() < high_watermark_) {
         return;
      }
      cv_below_watermark_.wait(lk, [&] { return stopped_ || q_.size() <= low_watermark_; });
   }

   void stop() {
     {
         std::lock_guard<std::mutex> lk(m_);
         stopped_ = true;
     }
     cv_not_empty_.notify_all();
     cv_below_watermark_.notify_all();
   }

   std::size_t size() const {
      std::lock_guard<std::mutex> lk(m_);
      return q_.size();
   }
private:
   mutable  std::mutex m_;
   std::condition_variable cv_;
   std::condition_variable cv_not_empty_;
   std::condition_variable cv_below_watermark_;
   std::deque<T> q_;
   bool stopped_ = false;

   std::size_t high_watermark_ = 0;
   std::size_t low_watermark_  =0;
};