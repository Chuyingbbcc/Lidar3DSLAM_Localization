//
// Created by chuchu on 3/12/26.
//
#pragma once
#include "DataType.h"
#include <ogrsf_frmts.h>
#include <map>
#include <memory>


class KeyFrame;
struct GPSData {
   double t_;
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
   OGRCoordinateTransformation* transform_;
   //Initial RTK, get attena angle, .etc
   bool InitialRTK(double fstLat, double fstLon);
   bool InitialGDAL(double fstLat, double fstLon);
   bool Convert2UTM(GPSData& data);
   bool GetPose(GPSData& data);
};

bool Interpolate_gps(const double qt, std::shared_ptr<KeyFrame>kf_ptr, std::map<double, GPSData>&gpsMap);




