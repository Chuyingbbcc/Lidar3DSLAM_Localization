#include "Lio/thread_pool.h"

#include <cstddef>
#include <mutex>
#include "point_cloud_utils.h"
#include "functional"

ThreadPool::ThreadPool(const size_t &thread_count, const size_t &max_queue): max_queue_(max_queue){
 start(thread_count);
}

ThreadPool::~ThreadPool() {
 stop();
}


bool ThreadPool::try_enqueue(std::function<void()> f) {
   std::function<void()>task(std::forward<std::function<void()>>(f));
   {
      std::lock_guard<std::mutex>lk(m_);
      if(stopped_.load())return false;

      if(max_queue_ >0  && tasks_.size() >= max_queue_)return false;
      tasks_.push(std::move(task));
   }
   cv_not_empty_.notify_one();
   return true;
}


bool ThreadPool::enqueue_blocking(std::function<void()> f) {
    std::function<void()>task(std::forward<std::function<void()>>(f));
    if(!task) return false;

    std::unique_lock<std::mutex> lk(m_);
    cv_not_full_.wait(lk, [&] {
       if (stopped_.load())return true;
       return (max_queue_ == 0) || (tasks_.size() <max_queue_);
    });
    if (stopped_.load())return  false;
    tasks_.push(std::move(task));
    cv_not_empty_.notify_one();
    return true;
}


bool ThreadPool::enqueue_drop_oldest(std::function<void()>f) {
    std::function<void()>task(std::forward<std::function<void()>>(f));
    if(!task) return false;

    std::lock_guard<std::mutex> lk(m_);
    if (stopped_.load()) return false;

    if (max_queue_ > 0 && tasks_.size() >= max_queue_) {
        // Drop the oldest queued task (LiDAR-friendly: keep latest)
        tasks_.pop();
    }

    tasks_.push(std::move(task));
    cv_not_empty_.notify_one();
    return true;
}

inline size_t ThreadPool::num_queued() const {
    std::lock_guard<std::mutex> lk(m_);
    return tasks_.size();
}

void ThreadPool::stop() {
  bool expected = false;
  if(!stopped_. compare_exchange_strong(expected, true))
      return;
  {
     std::lock_guard<std::mutex>lk(m_);
  }
  cv_not_empty_.notify_all();
  cv_not_full_.notify_all();

  for(auto& t: workers_) {
     if (t.joinable()) {
         t.join();
     }
  }
  workers_.clear();
}

void ThreadPool::start(std::size_t n_threads) {
 stopped_ = false;
 workers_.reserve(n_threads);

 for (std::size_t i = 0; i < n_threads; ++i) {
  workers_.emplace_back([this] {
    while (true) {
      std::function<void()> task;

      {
        std::unique_lock<std::mutex> lk(m_);
        cv_not_empty_.wait(lk, [&] {
          return stopped_.load() || !tasks_.empty();
        });

        if (stopped_.load() && tasks_.empty())
         return;

       task = std::move(tasks_.front());
       tasks_.pop();

       // We just freed one slot.
       cv_not_full_.notify_one();
     }

     task();
   }
 });
 }
}