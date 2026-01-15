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
   void push(T&& item) {
     {
        std::lock_guard<std::mutex>lk(m_);
        q_.push_back(std::move(item));
     }
      cv_.notify_one();
   }

   bool wait_pop(T& out) {
     std::unique_lock<std::mutex> lk(m_);
     cv_.wait(lk, [&] { return stopped_ ||!q_.empty(); });
     if (q_.empty()) return false;
     out = std::move(q_.front());
     q_.pop_front();
     return true;
   }

   void stop() {
     {
         std::lock_guard<std::mutex> lk(m_);
         stopped_ = true;
     }
     cv_.notify_all();
   }

   std::size_t size() const {
      std::lock_guard<std::mutex> lk(m_);
      return q_.size();
   }
private:
   mutable  std::mutex m_;
   std::condition_variable cv_;
   std::deque<T> q_;
   bool stopped_ = false;
};