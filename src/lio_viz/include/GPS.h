//
// Created by chuchu on 3/12/26.
//
#pragma once
#include "DataType.h"
#include <ogrsf_frmts.h>
#include <map>
#include <memory>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <yaml-cpp/yaml.h>


class KeyFrame;
struct GPSData {
   double t_;
   bool valid_ = true; // Invalid received fixes remain in the timeline as barriers.
   double lat_ = 0.0;
   double lon_ = 0.0;
   double alt_ = 0.0;

   double x_ =0.0;
   double y_ =0.0;
   double z_ =0.0;

   double heading_ =0.0;
   bool heading_valid_ = false;
   SE3d pose_;
};

struct GPS {
   bool initialized_ = false;
   SE3d TBG_ = SE3d();
   double antenna_angle_ = 0.0;
   double antenna_x_ = 0.0;
   double antenna_y_ = 0.0;
   OGRCoordinateTransformation* transform_ = nullptr;
   //Initial RTK, get attena angle, .etc
   bool InitialRTK(double fstLat, double fstLon);
   bool InitialGDAL(double fstLat, double fstLon);
   bool Convert2UTM(GPSData& data);
   bool GetPose(GPSData& data);
};

struct GnssOptions {
    double max_gap_sec = 1.0;
    double max_horizontal_std_m = 10.0;
};
GnssOptions loadGnssOptions(const YAML::Node& root);
bool gnssFixUsable(const sensor_msgs::msg::NavSatFix& fix, const GnssOptions& options);
bool hasGnssCoverage(double time, const std::map<double, GPSData>& samples, double max_gap_sec = 1.0);
size_t applyGnssCoverage(std::map<size_t, std::shared_ptr<KeyFrame>>& keyframes,
                         const std::map<double, GPSData>& samples, double max_gap_sec = 1.0);
bool Interpolate_gps(double qt, std::shared_ptr<KeyFrame> kf_ptr,
                     std::map<double, GPSData>& gpsMap, double max_gap_sec = 1.0);




