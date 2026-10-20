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
    void clear() {
        pts_.clear();
        has_bbox_ = false;
        min_x_ = min_y_ = min_z_ =  1e30;
        max_x_ = max_y_ = max_z_ = -1e30;
    }
    void reserve(size_t n) {pts_.reserve(n);}
    void resize(size_t n) {
        pts_.resize(n);
        rebuildBBox();
    }
    void setTime(const double t) {time_ = t;}

    void push_back(const Point& pt) {
        pts_.push_back(pt);
        updateBBox(pts_.back());
        updateBBox(pt);
    }

    template<class... Args>
    Point& emplace_back(Args&&... args) {
       pts_.emplace_back(Point{ std::forward<Args>(args)...});
       return pts_.back();
    }

    void append(const Point* src, std::size_t n) {
        if (n == 0) return;

        pts_.reserve(pts_.size() + n);

        for (std::size_t i = 0; i < n; ++i) {
            pts_.push_back(src[i]);
            updateBBox(src[i]);
        }
    }

    void append(std::shared_ptr<PointCloud>src_ptr, std::size_t n) {
        if (!src_ptr || n == 0) {
            return;
        }

        n = std::min(n, src_ptr->size());

        pts_.reserve(pts_.size() + n);

        for (std::size_t i = 0; i < n; ++i) {
            const Point& p = (*src_ptr)[i];
            pts_.push_back(p);
            updateBBox(p);
        }
     }

    bool hasBBox() const { return has_bbox_; }

    double minX() const { return min_x_; }
    double minY() const { return min_y_; }
    double minZ() const { return min_z_; }

    double maxX() const { return max_x_; }
    double maxY() const { return max_y_; }
    double maxZ() const { return max_z_; }
    private:
    std::vector<Point> pts_;
    double time_;

    bool has_bbox_ = false;
    double min_x_ =  1e30;
    double min_y_ =  1e30;
    double min_z_ =  1e30;
    double max_x_ = -1e30;
    double max_y_ = -1e30;
    double max_z_ = -1e30;

    void updateBBox(const Point& p) {
        if (!has_bbox_) {
            min_x_ = max_x_ = p.x;
            min_y_ = max_y_ = p.y;
            min_z_ = max_z_ = p.z;
            has_bbox_ = true;
            return;
        }

        min_x_ = std::min(min_x_, static_cast<double>(p.x));
        min_y_ = std::min(min_y_, static_cast<double>(p.y));
        min_z_ = std::min(min_z_, static_cast<double>(p.z));

        max_x_ = std::max(max_x_, static_cast<double>(p.x));
        max_y_ = std::max(max_y_, static_cast<double>(p.y));
        max_z_ = std::max(max_z_, static_cast<double>(p.z));
    }

    void rebuildBBox() {
        has_bbox_ = false;
        min_x_ = min_y_ = min_z_ =  1e30;
        max_x_ = max_y_ = max_z_ = -1e30;

        for (const auto& p : pts_) {
            updateBBox(p);
        }
    }
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
    if (msg_ptr->point_step == 0 || msg_ptr->width == 0) return false;
    if (msg_ptr->row_step < static_cast<uint64_t>(msg_ptr->width) * msg_ptr->point_step ||
        msg_ptr->data.size() < static_cast<uint64_t>(msg_ptr->row_step) * msg_ptr->height ||
        (down_sample && (!std::isfinite(voxel_size) || voxel_size <= 0))) return false;

    const auto fmap = buildFieldMap(msg_ptr);
    auto itx = fmap.find("x");
    auto ity = fmap.find("y");
    auto itz = fmap.find("z");
    if (itx == fmap.end() || ity == fmap.end() || itz == fmap.end()) {
        return false;
    }

    // Validate field layouts before accessing message bytes.
    for (const auto& name : {"x", "y", "z"}) {
        const auto& field = fmap.at(name);
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            static_cast<uint64_t>(field.offset) + sizeof(float) > msg_ptr->point_step) return false;
    }
    const auto intensity_field = fmap.find("intensity");
    if (intensity_field != fmap.end()) {
        const auto& field = intensity_field->second;
        size_t bytes = 0;
        switch (field.datatype) {
            case sensor_msgs::msg::PointField::FLOAT32: bytes = 4; break;
            case sensor_msgs::msg::PointField::FLOAT64: bytes = 8; break;
            case sensor_msgs::msg::PointField::UINT16: bytes = 2; break;
            case sensor_msgs::msg::PointField::UINT8: bytes = 1; break;
            default: return false;
        }
        if (static_cast<uint64_t>(field.offset) + bytes > msg_ptr->point_step) return false;
    }
    for (const auto& name : {"t", "time"}) {
        const auto it = fmap.find(name);
        if (it == fmap.end()) continue;
        const auto& field = it->second;
        const size_t bytes = field.datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8 :
                             field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 0;
        if (bytes && static_cast<uint64_t>(field.offset) + bytes > msg_ptr->point_step) return false;
    }
    // Supported sensor recordings use native little-endian fields.
    const uint16_t endian_probe = 1;
    const bool native_bigendian = *reinterpret_cast<const uint8_t*>(&endian_probe) == 0;
    if (msg_ptr->is_bigendian != native_bigendian) return false;
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
            const std::uint8_t* p = base + (i / msg_ptr->width) * msg_ptr->row_step + (i % msg_ptr->width) * msg_ptr->point_step;

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
                    case sensor_msgs::msg::PointField::FLOAT64:
                        intensity = static_cast<float>(readScalar<double>(p, fi.offset));
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

            out.emplace_back(x, y, z, std::isfinite(intensity) ? intensity : 0.0f, t);
        }
        return true;
    }

    const float inv = 1.0f / voxel_size;
    std::unordered_map<PtrVoxelKey, VoxelAccum, PtrVoxelHash> voxels;
    voxels.reserve(N / 4 + 1);

    for (std::size_t i = 0; i < N; ++i) {
        const std::uint8_t* p = base + (i / msg_ptr->width) * msg_ptr->row_step + (i % msg_ptr->width) * msg_ptr->point_step;

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
                case sensor_msgs::msg::PointField::FLOAT64:
                    intensity = static_cast<float>(readScalar<double>(p, fi.offset));
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
        a.si += std::isfinite(intensity) ? intensity : 0.0f;
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
// Keep XYZ files compatible with existing backend/evaluation tools. Intensity
// is stored as one float32 per point in a matching .intensity companion file.
inline bool loadPointCloudFromFile(std::string filename, std::shared_ptr<PointCloud> points,
                                   bool* has_intensity = nullptr) {
    points->clear();
    if (has_intensity) *has_intensity = false;
    std::ifstream xyz(filename, std::ios::binary | std::ios::ate);
    if (!xyz) return false;
    const auto bytes = xyz.tellg();
    if (bytes <= 0 || static_cast<size_t>(bytes) % (3 * sizeof(float))) return false;
    const size_t count = static_cast<size_t>(bytes) / (3 * sizeof(float));
    xyz.seekg(0);
    std::vector<float> intensities(count, 0.0f);
    std::ifstream intensity(filename + ".intensity", std::ios::binary | std::ios::ate);
    if (intensity) {
        if (intensity.tellg() != static_cast<std::streamoff>(count * sizeof(float))) {
            std::cerr << "Intensity count does not match cloud: " << filename << "\n";
            return false;
        }
        intensity.seekg(0);
        if (!intensity.read(reinterpret_cast<char*>(intensities.data()), count * sizeof(float))) return false;
        if (has_intensity) *has_intensity = true;
    }
    for (size_t i = 0; i < count; ++i) {
        float p[3];
        if (!xyz.read(reinterpret_cast<char*>(p), sizeof(p))) return false;
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) return false;
        points->emplace_back(p[0], p[1], p[2],
                            std::isfinite(intensities[i]) ? intensities[i] : 0.0f, 0.0f);
    }
    return true;
}

inline bool writePointCloudToFile(const std::string& path, std::shared_ptr<PointCloud> points) {
    std::ofstream xyz(path, std::ios::binary | std::ios::trunc);
    std::ofstream intensity(path + ".intensity", std::ios::binary | std::ios::trunc);
    if (!xyz || !intensity) return false;
    for (const auto& p : *points) {
        const float coords[3] = {p.x, p.y, p.z};
        const float value = std::isfinite(p.intensity) ? p.intensity : 0.0f;
        xyz.write(reinterpret_cast<const char*>(coords), sizeof(coords));
        intensity.write(reinterpret_cast<const char*>(&value), sizeof(value));
    }
    xyz.close();
    intensity.close();
    return bool(xyz) && bool(intensity);
}
#endif //POINTCLOUDUTILS_H
