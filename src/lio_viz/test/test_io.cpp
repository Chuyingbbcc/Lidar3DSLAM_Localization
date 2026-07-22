//
// Created by chuchu on 12/17/25.
//
#include <gtest/gtest.h>

#include  "io/ros_io_offline.h"
#include  "point_cloud_utils.h"
#include  <rclcpp/rclcpp.hpp>
#include  <std_msgs/msg/string.hpp>
#include "DataType.h"
#include <Eigen/Core>
#include "Align.h"
#include  "lidar_odometry.h"
#include  "Imu_data.h"
#include "KeyFrame.h"
#include "GPS.h"
#include   <thread>
#include   <map>
#include   <string>
#include <fstream>
#include <iomanip>

#include "../include/GPS.h"
#include "../include/Imu_data.h"
#include "../include/KeyFrame.h"
#include "../include/point_cloud_utils.h"
#include  "Lio/imu_buffer.h"
#include  "Lio/LidarDoneProcessedQueue.h"
#include "Lio/LioPipe.h"
#include "Lio/thread_pool.h"


class RosTestEnv: public::testing::Test {
protected:
    static void SetUpTestSuite() {
       if(!rclcpp::ok()) {
          int argc =0;
          rclcpp::init(argc, nullptr);
       }
    }
    static void TearDownTestSuite() {
      if (rclcpp::ok()) {
         rclcpp::shutdown();
      }
    }
};

// TEST(IO, TESTIO) {
//    const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";
//
//    RosIoOffline::IoOptions options;
//    options.bag_folder = ros_bag_path;
//    RosIoOffline io(options);
//
//    io.onImu([](const sensor_msgs::msg::Imu& imu, int64_t t_ns) {
//    (void)t_ns;
//    // push into imu buffer...
//     std::cout << "imu stamp " << imu.header.stamp.sec << "\n";
//    });
//    io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud, int64_t t_ns) {
//        PointCloud point_cloud;
//        pc2ToPointCloudXYZIT(cloud, point_cloud, 0.5, io.io_options_.down_sample_, true);
//
//       for (size_t i = 0; i < std::min<size_t>(point_cloud.size(), 5); ++i) {
//          const auto& p = point_cloud[i];
//          std::cout << "  [" << i << "] "
//                 << "x=" << p.x << " y=" << p.y << " z=" << p.z
//                 << " i=" << p.intensity << "\n" ;}
//    });
//    if(!io.go()) {
//       std::cerr<< "Failed rendering bag.\n";
//    }
//    rclcpp::shutdown();
//    EXPECT_TRUE(true);
// }
//
// TEST(IO, TEST_NDT_ALIGN) {
//    // //read ros 1 bag
//    // //sleep(10);
//    // //sleep(10);
//    // const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";
//    //
//    // RosIoOffline::IoOptions options;
//    // options.bag_folder = ros_bag_path;
//    // RosIoOffline io(options);
//    //
//    // PointCloud point_cloud;
//    // io.onImu([](const sensor_msgs::msg::Imu& imu, int64_t t_ns) {
//    // (void)t_ns;
//    // // push into imu buffer...
//    //  std::cout << "imu stamp " << imu.header.stamp.sec << "\n";
//    // });
//    // io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud, int64_t t_ns) {
//    //    pc2ToPointCloudXYZIT(cloud, point_cloud);
//    // });
//    // if(!io.go()) {
//    //    std::cerr<< "Failed rendering bag.\n";
//    // }
//    // rclcpp::shutdown();
//    // // convert it to the point cloud
//    // //give a customized rotation and translation
//    // float roll  = float(M_PI) / 70.0f;
//    // float pitch = float(M_PI) / 70.0f;
//    // float yaw  = float(M_PI) / 100.0f;
//    //
//    // Vec3f t(1.0f, 0.0f, 0.0f);
//    // Mat3f R = (
//    //   Eigen::AngleAxisf(yaw, Vec3f::UnitZ())*
//    //   Eigen::AngleAxisf(pitch, Vec3f::UnitY()) *
//    //      Eigen::AngleAxisf(roll,  Vec3f::UnitX()))
//    //         .toRotationMatrix();
//    //
//    // SE3f T(SO3f(R), t);
//    //
//    // std::vector<Vec3f> dstPtrs;
//    // std::vector<Vec3f> srcPtrs;
//    //
//    // PointCloud point_cloud2;
//    // for(const auto& p : point_cloud ) {
//    //    Vec3f ps(p.x, p.y, p.z);
//    //    Vec3f q = T*ps;
//    //    point_cloud2.emplace_back(q[0], q[1], q[2]);
//    // }
//    // //verify the implementation
//    // IncNDT ndt;
//    // std::shared_ptr<PointCloud>src_point_cloud_ptr = std::make_shared<PointCloud>(point_cloud);
//    // std::shared_ptr<PointCloud>dst_point_cloud_ptr = std::make_shared<PointCloud>(point_cloud2);
//    // ndt.AddCloud(src_point_cloud_ptr);
//    // ndt.SetSourceCloud(dst_point_cloud_ptr);
//    //
//    // SE3f initial = SE3f();
//    // if(!ndt.Align(initial)) {
//    //   std::cerr << "Failed aligning the ndt data!\n";
//    // }
//    //
//    // Mat3f rotation_matrix = initial.so3().matrix();
//    // Vec3f rpy = R.eulerAngles(2,1,0);
//    //
//    // float outYaw = rpy[0];
//    // float outPitch = rpy[1];
//    // float outRoll = rpy[2];
//    //
//    // EXPECT_TRUE((yaw-outYaw) < 1e-5f);
//    // EXPECT_TRUE((pitch-outPitch) < 1e-5f);
//    // EXPECT_TRUE((roll,-outRoll) < 1e-5f);
//    //sleep(30);
//    const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";
//
//    RosIoOffline::IoOptions options;
//    options.bag_folder = ros_bag_path;
//    RosIoOffline io(options);
//
//    LidarOdometry::LoOption op;
//    IncNDTOdometry inc_ndt_lo(op);
//
//    io.onImu([](const sensor_msgs::msg::Imu& imu, int64_t t_ns) {
//    (void)t_ns;
//    // push into imu buffer...
//     std::cout << "imu stamp " << imu.header.stamp.sec << "\n";
//    });
//    io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud, int64_t t_ns) {
//       PointCloud point_cloud;
//       SE3f init_pose = SE3f();
//       pc2ToPointCloudXYZIT(cloud, point_cloud, 0.5, io.io_options_.down_sample_);
//       std::shared_ptr<PointCloud>cloud_ptr = std::make_shared<PointCloud>(point_cloud);
//       inc_ndt_lo.AddCloud(cloud_ptr, init_pose, true);
//    });
//    if(!io.go()) {
//       std::cerr<< "Failed rendering bag.\n";
//    }
//    rclcpp::shutdown();
//    EXPECT_TRUE(true);
// }
//
// TEST(IO, TEST_INC_NDT_LO) {
//    //sleep(30);
//    const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";
//
//    RosIoOffline::IoOptions options;
//    options.bag_folder = ros_bag_path;
//    RosIoOffline io(options);
//
//    LidarOdometry::LoOption op;
//    IncNDTOdometry inc_ndt_lo(op);
//
//    io.onImu([](const sensor_msgs::msg::Imu& imu, int64_t t_ns) {
//    (void)t_ns;
//    // push into imu buffer...
//     std::cout << "imu stamp " << imu.header.stamp.sec << "\n";
//    });
//    io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud, int64_t t_ns) {
//       PointCloud point_cloud;
//       SE3f init_pose = SE3f();
//       pc2ToPointCloudXYZIT(cloud, point_cloud, 0.5, io.io_options_.down_sample_);
//       std::shared_ptr<PointCloud>cloud_ptr = std::make_shared<PointCloud>(point_cloud);
//       inc_ndt_lo.AddCloud(cloud_ptr, init_pose, true);
//    });
//    if(!io.go()) {
//       std::cerr<< "Failed rendering bag.\n";
//    }
//    rclcpp::shutdown();
//    EXPECT_TRUE(true);
// }


TEST(IO, TEST_LIDAR_PIPE) {
   //sleep(30);
   const std::string ros_bag_path = "/media/chuchu/Extreme Pro/unordered_ros_bag";

   RosIoOffline::IoOptions options;
   options.bag_folder = ros_bag_path;
   RosIoOffline io(options);

   LioOptions lio_options;

   ImuBuffer imu_buffer;
   LioPipe lio_pipe(3, 0.5, lio_options);

   size_t l_count =0;
   lio_pipe.onPreprocess(
      [](std::shared_ptr<sensor_msgs::msg::PointCloud2>msg_ptr, PointCloud& pc, const LioOptions& options) {
         pc2ToPointCloudXYZIT(msg_ptr, pc, 0.7, true);
      }
    );
   lio_pipe.start();

   io.onImu([&](const sensor_msgs::msg::Imu& imu_msg, double t_ns) {
      ImuData imu_data;
      toImuData(imu_data, imu_msg, t_ns);
      lio_pipe.pushImu(imu_data);
   });
   io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud_msg, double t_ns) {
      l_count++;
      //std:: cout << "The"<<l_count<<"th lidar data is "<<t_ns<<std::endl;

      lio_pipe.onLidarMsg(cloud_msg, t_ns);
   });
   if(!io.go()) {
      std::cerr<< "Failed rendering bag.\n";
   }
   //sleep(200);
   lio_pipe.notifyEndOfBag();
   std::cout<<"cosumer joined!"<<std::endl;
   lio_pipe.stopDrain();
   rclcpp::shutdown();
   std::cout<<"lidar pipe stopped!"<<std::endl;

   EXPECT_TRUE(true);
}

TEST(IO, TEST_KEY_FRMAES) {
   //sleep(20);
   const std::string ros_bag_path = "/media/chuchu/Extreme Pro/unordered_ros_bag";

   RosIoOffline::IoOptions options;
   options.bag_folder = ros_bag_path;
   RosIoOffline io(options);
   bool is_initial = false;
   GPS gps;
   size_t kf_idx  =0;
   size_t lidar_cnt =0;

   std::map<size_t, std::shared_ptr<KeyFrame>>key_frame_map;
   std::map<double, GPSData> gps_data_map;
   std::string dir = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/";

   io.onImu([&](const sensor_msgs::msg::Imu& imu_msg, double t_ns) {
   int a =5;
   });
   io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud_msg, double t_ns) {
      lidar_cnt++;

      //std:: cout << "lidar data is "<<t_ns<<std::endl;
      auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg);
      PointCloud pc;
      pc2ToPointCloudXYZIT( msg_ptr, pc, 0.5, true);
      std::shared_ptr<PointCloud> pc_ptr = std::make_shared<PointCloud>(pc);
      //create a key frame shared_ptr
      std::shared_ptr<KeyFrame> kf_ptr = std::make_shared<KeyFrame>(t_ns, kf_idx, SE3d(), pc_ptr);


      //after lio, write out the point cloud to file
      std::string cloud_path = dir + std::to_string(kf_idx) + ".pcd";
      std::cout<<kf_ptr->id_<<std::endl;
      //export the point cloiud
      if (!writePointCloudToFile(cloud_path, pc_ptr)) {
        std::cerr << "Failed to write point cloud: "<< cloud_path <<"\n";
      }
      kf_ptr->cloud_path_ = cloud_path;
      kf_ptr->cloud_size_ = kf_ptr->cloud_ptr_->size();
      kf_ptr->cloud_ptr_->clear();
      key_frame_map.emplace(kf_idx, kf_ptr);
      kf_idx++;
   });
   io.onGps(
      [&](const sensor_msgs::msg::NavSatFix& msg, double bag_time_ns) {
         //std::cout<< "gps time: "<<bag_time_ns<<std::endl;
         double lat  = static_cast<double>(msg.latitude);
         double lon  = static_cast<double>(msg.longitude);
         double alt  = static_cast<double>(msg.altitude);


         if(!is_initial) {
           auto res= gps.InitialRTK(lat,lon);
           ASSERT_TRUE(res);
         }
         GPSData gps_data;
         gps_data.t_ = bag_time_ns;
         gps_data.lat_ = lat;
         gps_data.lon_ = lon;
         gps_data.alt_ = alt;
         gps.Convert2UTM(gps_data);
         //std::cout<< std::setprecision(10)<<"x: "<<gps_data.x_<<"\n";
         //std::cout<< std::setprecision(10)<<"y: "<<gps_data.y_<<"\n";
         //std::cout<< std::setprecision(10)<<"z: "<<gps_data.z_<<"\n";
         gps.GetPose(gps_data);
         gps_data_map.emplace(bag_time_ns, gps_data);
      }
   );
   if(!io.go()) {
      std::cerr<< "Failed rendering bag.\n";
   }
   //sleep(200);
   rclcpp::shutdown();
   std::cout<< "There are "<< key_frame_map.size() << " keyframes in the bag.\n";

   //get gps for kfs
   for(auto it : key_frame_map) {
      Interpolate_gps(it.second->time_, it.second, gps_data_map);
   }
   std::string output_path = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/kf_output.txt";
   writeKeyFramesToFile(output_path, key_frame_map);

   std::map <size_t, std::shared_ptr<KeyFrame>> out_kf_map;
   loadKeyFrames(output_path, out_kf_map);

   //verify the kfs load and write
   ASSERT_EQ(out_kf_map.size(), key_frame_map.size());

   for(auto it : key_frame_map) {
      auto out_it = out_kf_map.find(it.first);
      ASSERT_TRUE(out_it != out_kf_map.end());
      auto out_kf = out_it->second;
      auto in_kf = it.second;
      SE3d dT = out_kf->rtk_pose_.inverse() * in_kf->rtk_pose_;
      double rot_err = dT.so3().log().norm();
      double trans_err = dT.translation().norm();

      ASSERT_LT(rot_err, 1e-6) << "Rotation mismatch at key " << it.first;
      ASSERT_LT(trans_err, 1e-6) << "Translation mismatch at key " << it.first;
      std::cout<< it.first << "\n";
      ASSERT_EQ(in_kf->cloud_size_, out_kf->cloud_size_);
   }

   std::cout<<"lidar pipe stopped!"<<std::endl;

   EXPECT_TRUE(true);
}


//--------------------imu--test-----------------------//
bool  isImuSampleValid(
    const ImuData& imu,
    double last_time) {
   // Check NaN and Inf.
   if (!std::isfinite(imu.t) ||
       !imu.acc.allFinite() ||
       !imu.gyro.allFinite()) {
        std::cout<<"NaN or Inf detected"<<std::endl;
        return false;
   }

   // Timestamp must increase.
   if (last_time >= 0.0 && imu.t <= last_time) {
      std::cout<< "Timestamp did not increase"<<std::endl;
      return false;
   }

   const double acc_norm = imu.acc.norm();
   const double gyro_norm = imu.gyro.norm();
   // Loose sanity limits for a hand-held platform.
   if (acc_norm > 50.0) {
     std::cout<<"Acceleration is too large"<<std::endl;
      return false;
   }

   if (gyro_norm > 10.0) {
      std::cout<<"Angular velocity is too large"<<std::endl;
      return false;
   }

   std::cout<<"Valid"<<std::endl;
   return true;
}

bool isImuWindowStable(const std::deque<ImuData>& imu_window) {
   constexpr std::size_t MIN_SAMPLES = 100;

   constexpr double GRAVITY = 9.81;
   constexpr double GRAVITY_TOLERANCE = 0.8;

   constexpr double MAX_MEAN_GYRO_NORM = 0.05;
   constexpr double MAX_GYRO_STD = 0.02;
   constexpr double MAX_ACC_NORM_STD = 0.25;

   if (imu_window.size() < MIN_SAMPLES) {
      std::cout<<"Not enough samples"<<std::endl;
      return false;
   }

   Vec3d mean_gyro = Vec3d::Zero();
   double mean_acc_norm = 0.0;
   double mean_gyro_norm = 0.0;

   for (const auto& imu : imu_window) {
      mean_gyro += imu.gyro;
      mean_acc_norm += imu.acc.norm();
      mean_gyro_norm += imu.gyro.norm();
   }

   const double sample_count =
       static_cast<double>(imu_window.size());


   mean_gyro /= sample_count;
   mean_acc_norm /= sample_count;
   mean_gyro_norm /= sample_count;

   double acc_norm_variance = 0.0;
   Vec3d gyro_variance = Vec3d::Zero();

   for (const auto& imu : imu_window) {
      const double acc_error =
          imu.acc.norm() - mean_acc_norm;

      acc_norm_variance += acc_error * acc_error;

      const Vec3d gyro_error =
          imu.gyro - mean_gyro;

      gyro_variance +=
          gyro_error.cwiseProduct(gyro_error);
   }

   acc_norm_variance /= sample_count;
   gyro_variance /= sample_count;

   const double acc_norm_std =
        std::sqrt(acc_norm_variance);

   const Vec3d gyro_std =
       gyro_variance.cwiseSqrt();

   const double max_gyro_std =
       gyro_std.maxCoeff();

   std::cout
       << "acc norm mean: " << mean_acc_norm
       << ", acc norm std: " << acc_norm_std
       << ", gyro norm mean: " << mean_gyro_norm
       << ", gyro std: " << gyro_std.transpose()
       << std::endl;

   if (std::abs(mean_acc_norm - GRAVITY) >
       GRAVITY_TOLERANCE) {
      std::cout<<"Acceleration norm is not close to gravity"<<std::endl;
      return false;
   }

   if (mean_gyro_norm > MAX_MEAN_GYRO_NORM) {
      std::cout<<"Platform is rotating"<<std::endl;
      return false;
   }

   if (max_gyro_std > MAX_GYRO_STD) {
      std::cout<<"Gyroscope data is noisy"<<std::endl;
      return false;
   }

   if (acc_norm_std > MAX_ACC_NORM_STD) {
      std::cout<<"Accelerometer data is noisy or moving"<<std::endl;
      return false;
   }
   std::cout<<"-------------------------"<<std::endl;
   std::cout<<"IMU is stable"<<std::endl;
   return true;
}

void processImuForTest(const ImuData& imu) {
   static std::deque<ImuData> imu_window;
   static double last_imu_time = -1.0;
   static int consecutive_stable_count = 0;
   static bool initialization_ready = false;

   constexpr int REQUIRED_STABLE_COUNT = 20;

   // Change this according to your timestamp unit.
   constexpr double WINDOW_DURATION = 1.5;

   // Step 1: Check the individual sample.
   if (!isImuSampleValid(imu, last_imu_time)) {
      std::cerr
          << "[INVALID IMU] t=" << imu.t
          << std::endl;

      imu_window.clear();
      consecutive_stable_count = 0;
      return;
   }
   last_imu_time = imu.t;

   // Step 2: Add valid sample to the window.
   imu_window.push_back(imu);

   // Keep only the latest 1.5 seconds.
   while (!imu_window.empty() &&
          imu.t - imu_window.front().t > WINDOW_DURATION) {
      imu_window.pop_front();
          }

   if (initialization_ready) {
      return;
   }

   // Step 3: Check whether this window is stable.
   const bool stable =
       isImuWindowStable(imu_window);

   if (stable) {
      ++consecutive_stable_count;

      std::cout
          << "[STABLE] count="
          << consecutive_stable_count
          << std::endl;
   } else {
      consecutive_stable_count = 0;

   }

   // Step 4: Only accept after several consecutive checks.
   if (consecutive_stable_count >= REQUIRED_STABLE_COUNT) {
      initialization_ready = true;

      std::cout
          << "====================================\n"
          << "IMU IS READY FOR INITIALIZATION\n"
          << "Stable window starts at: "
          << imu_window.front().t << '\n'
          << "Stable window ends at: "
          << imu_window.back().t << '\n'
          << "===================================="
          << std::endl;
   }
}

void saveImuToTxt(std::ofstream& fout, const ImuData& imu)
{
   if (!fout.is_open()) {
      return;
   }

   fout << std::fixed << std::setprecision(6)
        << imu.t << " "
        << imu.acc.x() << " "
        << imu.acc.y() << " "
        << imu.acc.z() << " "
        << imu.gyro.x() << " "
        << imu.gyro.y() << " "
        << imu.gyro.z() << " "
        << imu.acc.norm() << " "
        << imu.gyro.norm()
        << std::endl;
}

TEST(IO, TEST_IMU_VALID) {
   //sleep(20);
   const std::string ros_bag_path = "/media/chuchu/Extreme Pro/roof";

   RosIoOffline::IoOptions options;
   options.bag_folder = ros_bag_path;
   RosIoOffline io(ros_bag_path);
   bool is_initial = false;
   GPS gps;
   size_t kf_idx  =0;
   size_t lidar_cnt =0;

   std::map<size_t, std::shared_ptr<KeyFrame>>key_frame_map;
   std::map<double, GPSData> gps_data_map;
   std::string dir = "/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/";

   std::ofstream imu_file("/home/chuchu/Lidar3DSLAM_Localization/src/lio_viz/src/output_temp/imu_data.txt");

   imu_file << "# time "
            << "acc_x acc_y acc_z "
            << "gyro_x gyro_y gyro_z "
            << "acc_norm gyro_norm"
            << std::endl;
   try {
      io.onImu([&](const sensor_msgs::msg::Imu& imu_msg, double t_ns) {
       ImuData imu;
       toImuData(imu, imu_msg, t_ns, 9.80665, 1.0);
       processImuForTest(imu);
       saveImuToTxt(imu_file, imu);
      });
      if(!io.go()) {
         std::cerr<< "Failed rendering bag.\n";
      }
   }
   catch (const std::exception& e) {
     std::cerr<< "io exception: " << e.what() << std::endl;
   }
   //sleep(200);
   rclcpp::shutdown();
   std::cout<< "There are "<< key_frame_map.size() << " keyframes in the bag.\n";
   EXPECT_TRUE(true);
}