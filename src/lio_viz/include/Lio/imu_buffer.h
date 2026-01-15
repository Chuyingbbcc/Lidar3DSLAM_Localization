//
// Created by chuchu on 1/8/26.
//

#pragma once
#include <deque>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "DataType.h"
#include "Imu_data.h"

class ImuBuffer {
 public:
    ImuBuffer(const ImuBuffer& b)= delete;
    ImuBuffer& operator=(const ImuBuffer& b)= delete;
    ImuBuffer()= default;
    ~ImuBuffer()= default;

    void push(const ImuData& data);
    bool wait_until(double t, double timeout_sec =-1.0);
    void extract(std::vector<ImuData>& out, const double t0, const double t1 );
    void pop_segment(std::vector<ImuData>& out, double t0, double t1, double keep_prev = 0.1);

    void dropOlderThan(double t);

    bool empty() const;
    double latest_time()const;
private:
    std::deque<ImuData> buf_;
    mutable std::mutex m_;
    std::condition_variable cv_;
};

