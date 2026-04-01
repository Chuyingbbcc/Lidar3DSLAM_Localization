//
// Created by chuchu on 3/12/26.
//
#include "GPS.h"
#include "../include/GPS.h"

#include <ogrsf_frmts.h>
#include "DataType.h"
#include "KeyFrame.h"
#include "iostream"
#include "../include/DataType.h"

bool GPS::InitialRTK(double fstLat, double fstLon) {
    //get car world atte
    TBG_ = SE3d(SO3d::rotZ(antenna_angle_ * (M_PI /180.0)), Vec3d(antenna_x_, antenna_y_,0));
    if (!InitialGDAL(fstLat, fstLon)) {
        std::cerr<< "failed to initial gdal"<<std::endl;

    }
    return true;
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
    double x = data.lat_;
    double y = data.lon_;
    double z = data.alt_;

    auto res = transform_->Transform(1,&x,&y,&z);
    if(!res) {
        return false;
    }
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

bool Interpolate_gps(const double qt, std::shared_ptr<KeyFrame>kf_ptr, std::map<double, GPSData> &gpsMap) {
    if(gpsMap.size() == 0) {
       return false;
    }
    // get the lower bound
    auto it_right = gpsMap.lower_bound(qt);
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

    double alpha  = qt - t0 /t1 - t0;

    alpha = std::max(0.0, std::min(alpha, 1.0));
    SE3d delta = pose0.inverse()*pose1;
    SE3d pose2 = pose0 * SE3d::exp(alpha * delta.log());
    kf_ptr->rtk_pose_=  pose2;
    return true;
}