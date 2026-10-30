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
#include <limits>
#include <cstdio>
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
    float t;  // Relative seconds from scan start when per-point timing is available.
    int64_t offset_ns = 0;  // Authoritative point acquisition offset.
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
        has_point_timing_ = false;
        deskewed_ = false;
        scan_start_ns_ = scan_end_ns_ = pose_time_ns_ = 0;
        time_ = 0;
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
    bool hasPointTiming() const { return has_point_timing_; }
    bool isDeskewed() const { return deskewed_; }
    void markDeskewed() {
        if (!has_point_timing_) throw std::logic_error("Deskew metadata requires acquisition timing");
        deskewed_ = true;
    }
    int64_t scanStartNs() const { return scan_start_ns_; }
    int64_t scanEndNs() const { return scan_end_ns_; }
    int64_t poseTimeNs() const { return pose_time_ns_; }
    void setScanTiming(int64_t start_ns, int64_t end_ns) {
        if (start_ns < 0 || end_ns < start_ns) throw std::invalid_argument("Invalid scan interval");
        scan_start_ns_ = start_ns;
        scan_end_ns_ = pose_time_ns_ = end_ns;
        has_point_timing_ = true;
        deskewed_ = false;
        time_ = static_cast<double>(pose_time_ns_) * 1e-9;
    }

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
    double time_ = 0;
    bool has_point_timing_ = false;
    bool deskewed_ = false;
    int64_t scan_start_ns_ = 0, scan_end_ns_ = 0, pose_time_ns_ = 0;

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
   PointXYZIT timed_point{};
   int count = 0;
   int render_idx_ = -1;

};

inline bool pc2ToPointCloudXYZIT(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr,
    PointCloud& out,
    float voxel_size,
    bool down_sample,
    bool skip_nan = true,
    bool exclude_x_band = false,
    float excluded_x_min = 0.0f,
    float excluded_x_max = 0.0f)
{
    const double fallback_time = out.get_time();
    out.clear();
    if (msg_ptr->point_step == 0 || msg_ptr->width == 0) return false;
    if (msg_ptr->row_step < static_cast<uint64_t>(msg_ptr->width) * msg_ptr->point_step ||
        msg_ptr->data.size() < static_cast<uint64_t>(msg_ptr->row_step) * msg_ptr->height ||
        (down_sample && (!std::isfinite(voxel_size) || voxel_size <= 0)) ||
        (exclude_x_band && (!std::isfinite(excluded_x_min) || !std::isfinite(excluded_x_max) ||
                            excluded_x_max <= excluded_x_min))) return false;

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
    for (const auto& name : {"t", "time", "timestamp"}) {
        const auto it = fmap.find(name);
        if (it == fmap.end()) continue;
        const auto& field = it->second;
        const size_t bytes = field.datatype == sensor_msgs::msg::PointField::FLOAT64 ? 8 :
                             field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 0;
        if (!bytes || static_cast<uint64_t>(field.offset) + bytes > msg_ptr->point_step) return false;
        if (std::string(name) == "timestamp" && field.datatype != sensor_msgs::msg::PointField::FLOAT64) return false;
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
    const int64_t header_ns = static_cast<int64_t>(msg_ptr->header.stamp.sec) * 1000000000LL +
                              msg_ptr->header.stamp.nanosec;
    out.setTime(header_ns != 0 ? static_cast<double>(header_ns) * 1e-9 : fallback_time);
    const auto timestamp = fmap.find("timestamp");
    const bool timed = timestamp != fmap.end();
    // This recording's timestamp field is absolute nanoseconds, not seconds.
    // Inspect every raw point before crop/voxel filtering; storage order may differ from time order.
    if (timed) {
        if (N == 0) return false;
        int64_t start_ns = std::numeric_limits<int64_t>::max(), end_ns = 0;
        for (size_t i = 0; i < N; ++i) {
            const auto* p = base + (i / msg_ptr->width) * msg_ptr->row_step + (i % msg_ptr->width) * msg_ptr->point_step;
            const double raw = readScalar<double>(p, timestamp->second.offset);
            if (!std::isfinite(raw) || raw < 0 || raw >= static_cast<double>(std::numeric_limits<int64_t>::max())) return false;
            const int64_t ns = static_cast<int64_t>(std::llround(raw));
            start_ns = std::min(start_ns, ns);
            end_ns = std::max(end_ns, ns);
        }
        out.setScanTiming(start_ns, end_ns);
    }
    auto point_offset = [&](const uint8_t* p) -> int64_t {
        return timed ? static_cast<int64_t>(std::llround(readScalar<double>(p, timestamp->second.offset))) - out.scanStartNs() : 0;
    };


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

            if (z < 0 || x < -7 || x > 7 || y < -7 || y > 7 || z > 10 ||
                (exclude_x_band && x > excluded_x_min && x < excluded_x_max)) {
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
            if (timed) {
                t = static_cast<float>(point_offset(p) * 1e-9);
            } else if (has_t) {
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

            out.emplace_back(x, y, z, std::isfinite(intensity) ? intensity : 0.0f, t).offset_ns = point_offset(p);
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

        if (z < 0 || x < -7 || x > 7 || y < -7 || y > 7 || z > 10 ||
            (exclude_x_band && x > excluded_x_min && x < excluded_x_max)) {
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
        if (timed) {
            t = static_cast<float>(point_offset(p) * 1e-9);
        } else if (has_t) {
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
        if (timed && a.count == 0) {
            a.timed_point = PointXYZIT{x, y, z, std::isfinite(intensity) ? intensity : 0.0f, t, point_offset(p)};
        }
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

        // Keep an actual measurement and its acquisition time together.
        // Centroids mix acquisition poses; timed scans use the first point in each voxel.
        if (timed) { out.push_back(a.timed_point); continue; }
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
// Optional .timing companion: LIOTIME2, int64 start/end/reference nanoseconds,
// uint64 point count/flags, then int64 offsets. Flag bit 0 means deskewed.
// Native little-endian encoding; the reader also accepts legacy LIOTIME1.
inline bool writePointTiming(const std::string& path, const PointCloud& points) {
    if (!points.hasPointTiming()) {
        // A legacy rewrite must not leave a stale timing companion behind.
        std::ifstream stale(path + ".timing");
        if (!stale) return true;
        stale.close();
        return std::remove((path + ".timing").c_str()) == 0;
    }
    std::ofstream file(path + ".timing", std::ios::binary | std::ios::trunc);
    const int64_t times[] = {points.scanStartNs(), points.scanEndNs(), points.poseTimeNs()};
    const uint64_t count = points.size();
    const uint64_t flags = points.isDeskewed() ? 1 : 0;
    file.write("LIOTIME2", 8);
    file.write(reinterpret_cast<const char*>(times), sizeof(times));
    file.write(reinterpret_cast<const char*>(&count), sizeof(count));
    file.write(reinterpret_cast<const char*>(&flags), sizeof(flags));
    for (const auto& p : points) file.write(reinterpret_cast<const char*>(&p.offset_ns), sizeof(p.offset_ns));
    file.close();
    return bool(file);
}

inline bool loadPointTiming(const std::string& path, PointCloud& points) {
    std::ifstream file(path + ".timing", std::ios::binary | std::ios::ate);
    if (!file) return true;  // Legacy XYZ files have no acquisition timing.
    const auto bytes = file.tellg();
    file.seekg(0);
    char magic[8];
    int64_t times[3];
    uint64_t count;
    if (!file.read(magic, 8) ||
        (std::memcmp(magic, "LIOTIME1", 8) && std::memcmp(magic, "LIOTIME2", 8)) ||
        !file.read(reinterpret_cast<char*>(times), sizeof(times)) ||
        !file.read(reinterpret_cast<char*>(&count), sizeof(count))) return false;
    uint64_t flags = 0;
    const bool version2 = std::memcmp(magic, "LIOTIME2", 8) == 0;
    if (version2 && (!file.read(reinterpret_cast<char*>(&flags), sizeof(flags)) || (flags & ~uint64_t(1)))) return false;
    if (count != points.size() || bytes != static_cast<std::streamoff>((version2 ? 48 : 40) + points.size() * sizeof(int64_t)) ||
        times[0] < 0 || times[1] < times[0] || times[2] != times[1]) return false;
    std::vector<int64_t> offsets(count);
    if (!file.read(reinterpret_cast<char*>(offsets.data()), count * sizeof(int64_t))) return false;
    for (auto offset : offsets) if (offset < 0 || offset > times[1] - times[0]) return false;
    points.setScanTiming(times[0], times[1]);
    if (flags & 1) points.markDeskewed();
    for (size_t i = 0; i < count; ++i) {
        points[i].offset_ns = offsets[i];
        points[i].t = static_cast<float>(offsets[i] * 1e-9);
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
    return loadPointTiming(filename, *points);
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
    return bool(xyz) && bool(intensity) && writePointTiming(path, *points);
}
#endif //POINTCLOUDUTILS_H
