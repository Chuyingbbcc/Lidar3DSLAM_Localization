// common.h
//
// Created by chuchu on 12/12/25.
//

#ifndef COMMON_H
#define COMMON_H

#include <cstddef>
#include <initializer_list>

// Generic N-dimensional point
template<typename T, std::size_t N>
class Point {
public:
    T data[N];

    constexpr Point() : data{} {}

    Point(std::initializer_list<T> init) {
        std::size_t i = 0;
        for (T v : init) {
            if (i < N) {
                data[i++] = v;
            } else {
                break;
            }
        }
        for (; i < N; ++i) {
            data[i] = T{};
        }
    }

    constexpr T& operator[](std::size_t i) { return data[i]; }
    constexpr const T& operator[](std::size_t i) const { return data[i]; }
    constexpr std::size_t size() const { return N; }
};

#endif // COMMON_H
