//
// Created by chuchu on 3/12/26.
//
#include "GPS.h"
#include "../include/GPS.h"

#include <ogrsf_frmts.h>
#include "DataType.h"
#include "KeyFrame.h"
#include "iostream"
#include <cmath>
#include <stdexcept>
#include "../include/DataType.h"

bool GPS::InitialRTK(double fstLat, double fstLon) {
    //get car world atte
    TBG_ = SE3d(SO3d::rotZ(antenna_angle_ * (M_PI /180.0)), Vec3d(antenna_x_, antenna_y_,0));
    initialized_ = InitialGDAL(fstLat, fstLon);
    return initialized_;
}

bool GPS::InitialGDAL(double fstLat, double fstLon) {
    OGRSpatialReference srcSRS, dstSRS;
    srcSRS.SetWellKnownGeogCS("WGS84");

    int utmZone = static_cast<int>((fstLon+180)/6) +1;
    bool isNorth = (fstLat >=0);
    dstSRS.SetUTM(utmZone, isNorth);
    dstSRS.SetWellKnownGeogCS("WGS84");

    OGRCoordinateTransformation* transform = OGRCreateCoordinateTransformation(&srcSRS, &dstSRS);
    if(!transform) {
        std::cerr<< "Fail to create georeference transform"<<std::endl;
        return false;
    }
    transform_ = transform;
    return true;
}

bool GPS::Convert2UTM(GPSData &data) {
    if (!initialized_ || !transform_) return false;
    double x = data.lat_;
    double y = data.lon_;
    double z = data.alt_;

    auto res = transform_->Transform(1,&x,&y,&z);
    if(!res) {
        return false;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    data.x_ = x;
    data.y_ = y;
    data.z_ = z;
    return true;
}

bool GPS::GetPose(GPSData &data) {
    //global to body
  SE3d TGB = TBG_.inverse();
    // world to  gobal
    double heading = 0.0;
    // if(data.mHeadingValid) {
    //     heading = (90.0 - data.mHeading)* (M_PI /180.0);
    // }
    SE3d TWG(SO3d::rotZ(heading), Vec3d(data.x_, data.y_, data.z_));
    //SE3 TWG(SO3(),Vector3d(data.mX, data.mY, data.mZ));
    SE3d TWB = TWG *TGB;
    if(!data.heading_valid_) {
        data.pose_ = SE3d(SO3d(), TWB.translation());
    }
    else {
        data.pose_ = TWB;
    }
    return true;
}

GnssOptions loadGnssOptions(const YAML::Node& root) {
    GnssOptions options;
    const auto cfg = root["gnss"];
    if (cfg) {
        if (cfg["max_interpolation_gap_sec"]) options.max_gap_sec = cfg["max_interpolation_gap_sec"].as<double>();
        if (cfg["max_horizontal_std_m"]) options.max_horizontal_std_m = cfg["max_horizontal_std_m"].as<double>();
    }
    if (!std::isfinite(options.max_gap_sec) || options.max_gap_sec <= 0 ||
        !std::isfinite(options.max_horizontal_std_m) || options.max_horizontal_std_m <= 0)
        throw std::invalid_argument("GNSS gap and horizontal uncertainty limits must be finite and positive");
    return options;
}

bool gnssFixUsable(const sensor_msgs::msg::NavSatFix& fix, const GnssOptions& options) {
    if (fix.status.status < 0 || !std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
        !std::isfinite(fix.altitude) || std::abs(fix.latitude) > 90 || std::abs(fix.longitude) > 180)
        return false;
    // Unknown covariance is not an accuracy claim; retain status/coordinate checks.
    if (fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        for (double value : fix.position_covariance) if (!std::isfinite(value)) return false;
        if (fix.position_covariance[0] < 0 || fix.position_covariance[4] < 0 || fix.position_covariance[8] < 0)
            return false;
        const double limit = options.max_horizontal_std_m * options.max_horizontal_std_m;
        if (fix.position_covariance[0] > limit || fix.position_covariance[4] > limit) return false;
    }
    return true;
}

bool hasGnssCoverage(double time, const std::map<double, GPSData>& samples, double max_gap_sec) {
    if (!std::isfinite(time) || !std::isfinite(max_gap_sec) || max_gap_sec <= 0 || samples.empty()) return false;
    const auto right = samples.lower_bound(time);
    if (right == samples.end()) return false;
    const auto usable = [](const GPSData& sample) { return sample.valid_ && sample.pose_.matrix().allFinite(); };
    if (right->first == time) return usable(right->second);
    if (right == samples.begin()) return false;
    const auto left = std::prev(right);
    return right->first - left->first <= max_gap_sec && usable(left->second) && usable(right->second);
}

size_t applyGnssCoverage(std::map<size_t, std::shared_ptr<KeyFrame>>& keyframes,
                         const std::map<double, GPSData>& samples, double max_gap_sec) {
    size_t rejected = 0;
    for (auto& entry : keyframes) {
        auto& kf = entry.second;
        if (!kf || !kf->rtk_valid_) continue;
        if (!kf->rtk_pose_.matrix().allFinite() || !hasGnssCoverage(kf->time_, samples, max_gap_sec)) {
            kf->rtk_valid_ = false;
            kf->rtk_inlier_ = false;
            kf->rtk_heading_valid_ = false;
            ++rejected;
        }
    }
    return rejected;
}

bool Interpolate_gps(double qt, std::shared_ptr<KeyFrame> kf_ptr, std::map<double, GPSData>& gpsMap, double max_gap_sec) {
    if (!kf_ptr || !hasGnssCoverage(qt, gpsMap, max_gap_sec)) return false;
    if(gpsMap.size() == 0) {
       return false;
    }
    // get the lower bound
    auto it_right = gpsMap.lower_bound(qt);
    if (it_right == gpsMap.end()) return false;
    if(it_right == gpsMap.begin()) {
       if(it_right->first == qt) {
          kf_ptr->rtk_pose_ =  it_right->second.pose_;
          return true;
       }
      return false;
    }
    if(it_right->first == qt) {
        kf_ptr->rtk_pose_ =  it_right->second.pose_;
        return true;
    }
    // and do linear interpolation
    auto it_left = std::prev(it_right);
    double t0 = it_left->first;
    double t1 = it_right->first;

    SE3d pose0 = it_left->second.pose_;
    SE3d pose1 =  it_right->second.pose_;

    double alpha = (qt - t0) / (t1 - t0);

    alpha = std::max(0.0, std::min(alpha, 1.0));
    SE3d delta = pose0.inverse()*pose1;
    SE3d pose2 = pose0 * SE3d::exp(alpha * delta.log());
    kf_ptr->rtk_pose_=  pose2;
    return true;
}