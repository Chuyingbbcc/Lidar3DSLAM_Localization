#pragma once


#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include "point_cloud_utils.h"


class LidarReorderBuffer {
public:
    using CloudPtr  =std::shared_ptr<PointCloud>;
    explicit LidarReorderBuffer(double slack_sec = 0.1);
    ~LidarReorderBuffer();

    LidarReorderBuffer(const LidarReorderBuffer&) = delete;
    LidarReorderBuffer& operator=(const LidarReorderBuffer&) = delete;

    void onRawSeen(double t1);                 // IO thread: update watermark
    void pushProcessed(PointCloud& scan);  // worker thread(s): push into heap

    bool waitPopOrdered(CloudPtr& out);   // consumer thread: ordered scans
    void waitUntilDone();
    void waitIfReadyTooLarge();

    void stop();
    void closeInput();

private:
    struct HeapItem {
        double t1;
        std::uint64_t tie;
        PointCloud scan;
    };
    struct Cmp {
        bool operator()(const HeapItem& a, const HeapItem& b) const {
            if (a.t1 != b.t1) return a.t1 > b.t1;   // min-heap
            return a.tie > b.tie;
        }
    };

    void commitLoop();


private:
    const double slack_sec_;
    std::atomic<bool> running_{false};

    // watermark
    std::mutex m_wm_;
    double watermark_t1_{-1.0};

    // heap
    std::mutex m_heap_;
    std::condition_variable cv_heap_;
    std::priority_queue<HeapItem, std::vector<HeapItem>, Cmp> heap_;
    std::atomic<std::uint64_t> tie_counter_{0};

    // ordered ready queue
    std::mutex m_ready_;
    std::condition_variable cv_ready_;
    std::deque<CloudPtr> ready_;

    //
    std::condition_variable cv_backpressure_;
    size_t high_watermark_ = 100;
    size_t low_watermark_ = 60;

    bool input_closed_ = false;
    bool done_ =false;

    std::thread commit_thread_;
};


