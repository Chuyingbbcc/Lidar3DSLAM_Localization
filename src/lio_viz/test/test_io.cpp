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
#include   <thread>

#include <thread>

#include  "Lio/imu_buffer.h"
#include  "Lio/LidarDoneProcessedQueue.h"
#include "Lio/LidarPreProcessing.h"
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
   sleep(20);
   const std::string ros_bag_path = "/media/chuchu/Extreme Pro/ros_bag_example";

   RosIoOffline::IoOptions options;
   options.bag_folder = ros_bag_path;
   RosIoOffline io(options);

   LioOptions lio_options;

   ImuBuffer imu_buffer;
   LidarPreProcessing lidar_pre_pipe(3);

   std::thread consumer([&]{lidar_pre_pipe.postProcess();});

   io.onImu([&](const sensor_msgs::msg::Imu& imu_msg, int64_t t_ns) {
      ImuData imu_data;
      toImuData(imu_data, imu_msg, t_ns);
      imu_buffer.push(imu_data);
   });
   io.onLidar([&](const sensor_msgs::msg::PointCloud2& cloud_msg, int64_t t_ns) {
      lidar_pre_pipe.onPreprocess(
        [](std::shared_ptr<sensor_msgs::msg::PointCloud2>msg_ptr, PointCloud& pc, const LioOptions& options) {
           pc2ToPointCloudXYZIT(msg_ptr, pc, options.voxel_size, options.voxel_down_sample);
        }
      );
      lidar_pre_pipe.onLidarMsg(cloud_msg, t_ns, lio_options);
   });
   if(!io.go()) {
      std::cerr<< "Failed rendering bag.\n";
   }
   rclcpp::shutdown();
   consumer.join();
   std::cout<<"cosumer joined!"<<std::endl;
   lidar_pre_pipe.stop();
   std::cout<<"lidar pipe stopped!"<<std::endl;

   EXPECT_TRUE(true);
}