//
// Created by chuchu on 12/19/25.
//

#ifndef POINTCLOUDUTILS_H
#define POINTCLOUDUTILS_H


#include <vector>
#include <memory>
#include <algorithm>
#include <utility>
#include <unordered_set>

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <fstream>


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
    double get_time() const {
      return time_;
    }
    //iteration
    auto begin() {return pts_.begin();}
    auto end() {return pts_.end();}
    auto begin() const {return pts_.begin();}
    auto end() const {return pts_.end();}

    //modifiers
    void clear() {pts_.clear();}
    void reserve(size_t n) {pts_.reserve(n);}
    void resize(size_t n) {pts_.resize(n);}
    void setTime(const double t) {time_ = t;}

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
    double time_;
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
buildFieldMap(const std::shared_ptr<sensor_msgs::msg::PointCloud2>msg) {
   std::unordered_map<std::string, FieldInfo> m;
   m.reserve(msg->fields.size());
   for (const auto& f: msg->fields) {
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

struct PtrVoxelKey {
  int32_t x, y,z;
  bool operator ==(const PtrVoxelKey& o) const {
    return o.x == x && o.y == y && o.z == z;
  }
};

struct PtrVoxelHash {
  size_t operator () (const PtrVoxelKey& k) const noexcept {
     uint64_t h = 1469598103934665603ull;
     auto mix = [&](uint32_t v){ h^=v, h*=1099511628211ull;};
     mix((uint32_t)k.x);
     mix((uint32_t)k.y);
     mix((uint32_t)k.z);
     return (size_t)h;
  }
};
struct VoxelAccum {
   double sx = 0.0, sy = 0.0, sz = 0.0;
   double si = 0.0, st = 0.0;
   int count = 0;
   int render_idx_ = -1;

};
// inline bool pc2ToPointCloudXYZIT( const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr,
//     PointCloud& out,
//     float voxel_size,
//     bool down_sample,
//     bool skip_nan = true){
//     out.clear();
//     if(msg_ptr->point_step ==0) return false;
//
//     const auto fmap =buildFieldMap(msg_ptr);
//     auto itx = fmap.find("x");
//     auto ity = fmap.find("y");
//     auto itz = fmap.find("z");
//     if (itx == fmap.end() || ity == fmap.end() || itz == fmap.end()) {
//         return false;
//     }
//     const bool has_intensity = (fmap.find("intensity") != fmap.end());
//     const bool has_t = (fmap.find("t") != fmap.end()) || (fmap.find("time") != fmap.end());
//     const auto itI = fmap.find("intensity");
//     const auto itT = fmap.find("t");
//     const auto itTime = fmap.find("time");
//
//     const std::size_t N = static_cast<std::size_t>(msg_ptr->width) * static_cast<std::size_t>(msg_ptr->height);
//     out.reserve(N);
//
//     const float inv = 1.0f / voxel_size;
//     std::unordered_set<PtrVoxelKey, PtrVoxelHash>seen;
//
//     const std::uint8_t* base = msg_ptr->data.data();
//     for (std::size_t i = 0; i < N; ++i) {
//        const std::uint8_t* p = base + i*msg_ptr->point_step;
//
//        float x = readScalar<float>(p, itx->second.offset);
//        float y = readScalar<float>(p, ity->second.offset);
//        float z = readScalar<float>(p, itz->second.offset);
//        if (skip_nan) {
//           if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z)))continue;
//        }
//
//        PtrVoxelKey key{
//         (int32_t)std::floor(x *inv),
//            (int32_t)std::floor(y *inv),
//            (int32_t)std::floor(z *inv),
//        };
//
//        if(down_sample && !seen.insert(key).second)continue;
//        if(z<-1 || x< -40 || x> 40 || y< -40 || y>40) {
//          continue;
//        }
//
//        float intensity = 0.f;
//        if (has_intensity) {
//          const auto& fi  = itI->second;
//          switch (fi.datatype) {
//             case sensor_msgs::msg::PointField::FLOAT32:
//                intensity = readScalar<float>(p, fi.offset);
//                break;
//             case sensor_msgs::msg::PointField::UINT16:
//                intensity = static_cast<float>(readScalar<std::uint16_t>(p, fi.offset));
//                break;
//             case sensor_msgs::msg::PointField::UINT8:
//                intensity = static_cast<float>(readScalar<std::uint8_t>(p, fi.offset));
//                break;
//             default:
//                // unknown type -> leave 0
//                break;
//          }
//        }
//        float t = 0.f;
//        if (has_t) {
//           const FieldInfo* ft  = nullptr;
//           if (itT != fmap.end()) ft= &itT->second;
//           else if (itTime != fmap.end()) ft = &itTime->second;
//           if (ft) {
//              if (ft->datatype == sensor_msgs::msg::PointField::FLOAT32) {
//                 t = readScalar<float>(p, ft->offset);
//              } else if (ft->datatype == sensor_msgs::msg::PointField::FLOAT64) {
//                 t = static_cast<float>(readScalar<double>(p, ft->offset));
//              }
//           }
//        }
//        out.emplace_back(x,y,z,intensity,t);
//     }
//    //std::cout<<N<< " points read from msg. "<< out.size()<< " stored"<<std::endl;
//    return true;
// }
inline bool pc2ToPointCloudXYZIT(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr,
    PointCloud& out,
    float voxel_size,
    bool down_sample,
    bool skip_nan = true)
{
    out.clear();
    if (msg_ptr->point_step == 0) return false;

    const auto fmap = buildFieldMap(msg_ptr);
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

    const std::size_t N =
        static_cast<std::size_t>(msg_ptr->width) *
        static_cast<std::size_t>(msg_ptr->height);

    const std::uint8_t* base = msg_ptr->data.data();

    if (!down_sample) {
        out.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            const std::uint8_t* p = base + i * msg_ptr->point_step;

            float x = readScalar<float>(p, itx->second.offset);
            float y = readScalar<float>(p, ity->second.offset);
            float z = readScalar<float>(p, itz->second.offset);

            if (skip_nan && !(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
                continue;
            }

            if (z < -1 || x < -40 || x > 40 || y < -40 || y > 40 || z>5) {
                continue;
            }

            float intensity = 0.f;
            if (has_intensity) {
                const auto& fi = itI->second;
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
                        break;
                }
            }

            float t = 0.f;
            if (has_t) {
                const FieldInfo* ft = nullptr;
                if (itT != fmap.end()) ft = &itT->second;
                else if (itTime != fmap.end()) ft = &itTime->second;

                if (ft) {
                    if (ft->datatype == sensor_msgs::msg::PointField::FLOAT32) {
                        t = readScalar<float>(p, ft->offset);
                    } else if (ft->datatype == sensor_msgs::msg::PointField::FLOAT64) {
                        t = static_cast<float>(readScalar<double>(p, ft->offset));
                    }
                }
            }

            out.emplace_back(x, y, z, intensity, t);
        }
        return true;
    }

    const float inv = 1.0f / voxel_size;
    std::unordered_map<PtrVoxelKey, VoxelAccum, PtrVoxelHash> voxels;
    voxels.reserve(N / 4 + 1);

    for (std::size_t i = 0; i < N; ++i) {
        const std::uint8_t* p = base + i * msg_ptr->point_step;

        float x = readScalar<float>(p, itx->second.offset);
        float y = readScalar<float>(p, ity->second.offset);
        float z = readScalar<float>(p, itz->second.offset);

        if (skip_nan && !(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
            continue;
        }

        if (z < -1 || x < -40 || x > 40 || y < -40 || y > 40 || z>5) {
            continue;
        }

        float intensity = 0.f;
        if (has_intensity) {
            const auto& fi = itI->second;
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
                    break;
            }
        }

        float t = 0.f;
        if (has_t) {
            const FieldInfo* ft = nullptr;
            if (itT != fmap.end()) ft = &itT->second;
            else if (itTime != fmap.end()) ft = &itTime->second;

            if (ft) {
                if (ft->datatype == sensor_msgs::msg::PointField::FLOAT32) {
                    t = readScalar<float>(p, ft->offset);
                } else if (ft->datatype == sensor_msgs::msg::PointField::FLOAT64) {
                    t = static_cast<float>(readScalar<double>(p, ft->offset));
                }
            }
        }

        PtrVoxelKey key{
            static_cast<int32_t>(std::floor(x * inv)),
            static_cast<int32_t>(std::floor(y * inv)),
            static_cast<int32_t>(std::floor(z * inv))
        };

        VoxelAccum& a = voxels[key];
        a.sx += x;
        a.sy += y;
        a.sz += z;
        a.si += intensity;
        a.st += t;
        a.count += 1;
    }

    out.reserve(voxels.size());
    for (const auto& kv : voxels) {
        const VoxelAccum& a = kv.second;
        if (a.count <= 0) continue;

        const float inv_count = 1.0f / static_cast<float>(a.count);
        out.emplace_back(
            static_cast<float>(a.sx * inv_count),
            static_cast<float>(a.sy * inv_count),
            static_cast<float>(a.sz * inv_count),
            static_cast<float>(a.si * inv_count),
            static_cast<float>(a.st * inv_count)
        );
    }

    return true;
}
inline bool loadPointCloudFromFile(std::string filename, std::shared_ptr<PointCloud>pointPtr) {
   pointPtr->clear();
   std::fstream is(filename);
   if(!is) {
         std::cerr << "Failed to open file\n";
         return false;
   }
   //get file size in bytes
   is.seekg(0, std::ios::end);
   std::streampos nbytes = is.tellg();
   if(nbytes <=0) {
      std::cerr << "Empty file\n";
      return false;
   }
   is.seekg(0, std::ios::beg);
   std::size_t total_bytes  = static_cast<std::size_t>(nbytes);
   std::size_t num_floats   = total_bytes / sizeof(float);
   std::size_t fields_per_point = 3;
   // if (num_floats % 5 == 0) {
   //    fields_per_point = 5;
   // } else if (num_floats % 4 == 0) {
   //    fields_per_point = 4;
   // }

   //std::cout<<fields_per_point<<std::endl;
   std::size_t num_points = num_floats / fields_per_point;
   std::vector<float> buffer(num_floats);
   is.read(reinterpret_cast<char*>(buffer.data()), total_bytes);

   if (!is.good()) {
      std::cout <<  "Failed to read binary data" << std::endl;
      return false;
   }
   for (std::size_t i = 0; i < num_points; ++i) {
      const float x = buffer[i * fields_per_point + 0];
      const float y = buffer[i * fields_per_point + 1];
      const float z = buffer[i * fields_per_point + 2];
      pointPtr->emplace_back(x,y,z);
   }
   return true;
}

inline bool writePointCloudToFile(const std::string& out_path, std::shared_ptr<PointCloud>cloud_ptr) {
   //TODO: implement later
   //std::cout<<"SaveSubMap"<<std::endl;
   std::ofstream ofs(out_path.c_str(), std::ios::binary|std::ios::app);
   if(!ofs) {
      std::cerr<< "Can't open cloud file" << std::endl;
      return false;
   }
   //write in only one cloud file
   uint64_t n = cloud_ptr->size();
   for(int i=0; i< n; i++) {
      const auto& p = (*cloud_ptr)[i];
      ofs.write(reinterpret_cast<const char*>(&p.x), sizeof(float));
      ofs.write(reinterpret_cast<const char*>(&p.y), sizeof(float));
      ofs.write(reinterpret_cast<const char*>(&p.z), sizeof(float));
      //ofs.write(reinterpret_cast<const char*>(&p.intensity), sizeof(float));
   }
   uint64_t total_size = n*sizeof(float)*3;
   //std::cout<< "Written "<< total_size<< " of " <<n<<"points"<<std::endl;
   //open the file handler
   //write binary to the file
   return true;
}
#endif //POINTCLOUDUTILS_H
