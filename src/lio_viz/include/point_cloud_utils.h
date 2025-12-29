//
// Created by chuchu on 12/19/25.
//

#ifndef POINTCLOUDUTILS_H
#define POINTCLOUDUTILS_H


#include <vector>
#include <memory>
#include <algorithm>
#include <utility>

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <cmath>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

struct PointXYZIT {
    float x,y,z;
    float intensity;
    float t;
};

class PointCloud {
    public:
    using Point = PointXYZIT;
    //constructor:
    PointCloud() = default;
    explicit PointCloud (size_t size){pts_.reserve(size);}

    //size, capacity, empty
    std::size_t size() const {return pts_.size();}
    std::size_t capacity()const {return pts_.capacity();}
    bool empty()const {return pts_.empty();}

    //raw pointer
    Point* data() {return pts_.data();}
    const Point* data() const {return pts_.data();}

    //access
    Point& operator[] (std::size_t i){return pts_[i];}
    const Point& operator[] (std::size_t i) const {return pts_[i];}

    //iteration
    auto begin() {return pts_.begin();}
    auto end() {return pts_.end();}
    auto begin() const {return pts_.begin();}
    auto end() const {return pts_.end();}

    //modifiers
    void clear() {pts_.clear();}
    void reserve(size_t n) {pts_.reserve(n);}
    void resize(size_t n) {pts_.resize(n);}

    void push_back(const Point& pt) {pts_.push_back(pt);}

    template<class... Args>
    Point& emplace_back(Args&&... args) {
       pts_.emplace_back(Point{ std::forward<Args>(args)...});
       return pts_.back();
    }

    void append(const Point* src, std::size_t n) {
       if (n==0)return;
       const std::size_t old = pts_.size();
       pts_.resize(old+n);
       std::copy(src,src+n,pts_.begin()+old);
    }
    private:
    std::vector<Point> pts_;
};

class CloudSlice {
    public:
    using Point = PointXYZIT;

    CloudSlice() = default;
    CloudSlice(const Point* data, std::size_t n): data_{data}, size_{n} {}

    const Point* data() const{return data_;}
    std::size_t size() const {return size_;}
    bool empty() const {return size_ == 0;}

    const Point& operator[] (std::size_t i) const {return data_[i];}
    const Point* begin() const {return data_;}
    const Point* end() const {return data_ + size_;}

    CloudSlice sub(std::size_t offset, std::size_t count) const {
       if(offset> size_) offset = size_;
       count = std::min(count, size_ - offset);
       return CloudSlice(data_ + offset, count);
    }
    private:
    const Point* data_ = nullptr;
    std::size_t size_ = 0;
};

inline void merge (PointCloud& out, const PointCloud& a, const PointCloud& b) {
    out.clear();
    out.reserve(a.size() + b.size());
    out.append(a.data(), a.size());
    out.append(b.data(), b.size());
}


//----------------------------read point cloud from bag-----------------------------?//
struct FieldInfo {
   std::uint32_t offset =0;
   std::uint8_t datatype =0;
};

inline std::unordered_map<std::string, FieldInfo>
buildFieldMap(const sensor_msgs::msg::PointCloud2& msg) {
   std::unordered_map<std::string, FieldInfo> m;
   m.reserve(msg.fields.size());
   for (const auto& f: msg.fields) {
      m.emplace(f.name, FieldInfo{f.offset, f.datatype});
   }
   return m;
}

template <typename T>
inline T readScalar(const uint8_t* p, std::uint32_t offset) {
   T v;
   std::memcpy(&v, p + offset, sizeof(T));
   return v;
}

inline bool pc2ToPointCloudXYZIT( const sensor_msgs::msg::PointCloud2& msg,
    PointCloud& out,
    bool skip_nan = true){
    out.clear();
    if(msg.point_step ==0) return false;

    const auto fmap =buildFieldMap(msg);
    auto itx = fmap.find("x");
    auto ity = fmap.find("y");
    auto itz = fmap.find("z");
    if (itx == fmap.end() || ity == fmap.end() || itz == fmap.end()) {
        return false;
    }
    const bool has_intensity = (fmap.find("intensity") != fmap.end());
    const bool has_t = (fmap.find("t") != fmap.end()) || (fmap.find("time") != fmap.end());
    const auto itI = fmap.find("intensity");
    const auto itT = fmap.find("t");
    const auto itTime = fmap.find("time");

    const std::size_t N = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
    out.reserve(N);

    const std::uint8_t* base = msg.data.data();
    for (std::size_t i = 0; i < N; ++i) {
       const std::uint8_t* p = base + i*msg.point_step;

       float x = readScalar<float>(p, itx->second.offset);
       float y = readScalar<float>(p, ity->second.offset);
       float z = readScalar<float>(p, itz->second.offset);
       if (skip_nan) {
          if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z)))continue;
       }

       float intensity = 0.f;
       if (has_intensity) {
         const auto& fi  = itI->second;
         switch (fi.datatype) {
            case sensor_msgs::msg::PointField::FLOAT32:
               intensity = readScalar<float>(p, fi.offset);
               break;
            case sensor_msgs::msg::PointField::UINT16:
               intensity = static_cast<float>(readScalar<std::uint16_t>(p, fi.offset));
               break;
            case sensor_msgs::msg::PointField::UINT8:
               intensity = static_cast<float>(readScalar<std::uint8_t>(p, fi.offset));
               break;
            default:
               // unknown type -> leave 0
               break;
         }
       }
       float t = 0.f;
       if (has_t) {
          const FieldInfo* ft  = nullptr;
          if (itT != fmap.end()) ft= &itT->second;
          else if (itTime != fmap.end()) ft = &itTime->second;
          if (ft) {
             if (ft->datatype == sensor_msgs::msg::PointField::FLOAT32) {
                t = readScalar<float>(p, ft->offset);
             } else if (ft->datatype == sensor_msgs::msg::PointField::FLOAT64) {
                t = static_cast<float>(readScalar<double>(p, ft->offset));
             }
          }
       }
       out.emplace_back(x,y,z,intensity,t);
    }
   return true;
}
#endif //POINTCLOUDUTILS_H
