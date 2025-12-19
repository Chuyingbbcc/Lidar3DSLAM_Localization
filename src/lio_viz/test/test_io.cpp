//
// Created by chuchu on 12/17/25.
//
#include <gtest/gtest.h>

#include "../include/io/ros_io_offline.h"
#include  "io/ros_io_offline.h"


TEST(IO, TESTIO) {
    RosIoOffline::IoOptions options;
    RosIoOffline ros_io(options);
    EXPECT_TRUE(true);
}