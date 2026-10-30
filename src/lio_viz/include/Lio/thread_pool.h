//
// Created by chuchu on 1/9/26.
//
//
// Created by chuchu on 1/9/26.
//
#pragma once

#include <vector>
#include <thread>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>

class ThreadPool {
public:
    ThreadPool(const size_t& thread_count, const size_t& max_queue =64);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;


    bool try_enqueue(std::function<void()> f);


    bool enqueue_blocking(std::function<void()>f);

    bool enqueue_drop_oldest(std::function<void()> f);

    size_t num_queued() const;
    size_t max_queued() const { return max_queued_.load(); }
    size_t rejected() const { return rejected_.load(); }
    void stop();
private:
    void start(std::size_t n_threads);
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    mutable std::mutex m_;
    std::condition_variable cv_not_empty_;
    std::condition_variable cv_not_full_;
    std::atomic<bool> stopped_{false};

    std::size_t max_queue_;
    std::atomic<size_t> max_queued_{0};
    std::atomic<size_t> rejected_{0};
};



