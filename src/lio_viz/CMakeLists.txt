cmake_minimum_required(VERSION 3.5)
project(lio_viz)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lio_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GDAL REQUIRED)
find_package(g2o REQUIRED)
find_library(G2O_SOLVER_CSPARSE_LIB
        NAMES g2o_solver_csparse
        PATHS /usr/local/lib /usr/lib
        REQUIRED
)
find_path(CHOLMOD_INCLUDE_DIR
        NAMES cholmod.h
        PATHS
        /usr/include/suitesparse
        /usr/local/include/suitesparse
        /usr/include
        /usr/local/include
        REQUIRED
)
find_package(yaml-cpp REQUIRED)


# rosbag2 offline
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)


# ---- message types you will deserialize ----
find_package(sensor_msgs REQUIRED)
# Optional but common:
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# OpenGL & GLFW
find_package(JPEG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
find_package(glm REQUIRED)  # from libglm-dev


include_directories(
        include
        ${GLFW_INCLUDE_DIRS}
        3rd_party/sophus/sophus
)

#----------------glad----------------------------#
add_library(glad src/glad.c)
target_include_directories(glad PUBLIC include/glad )
target_include_directories(glad PUBLIC include/KHR)

SET(SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
#---------------g2o-----------#
add_library(pose_graph_optimizer
 ${SRC_PATH}/Slam/Optimization.cpp
)
target_include_directories(pose_graph_optimizer PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${CHOLMOD_INCLUDE_DIR}
)
# Only this target needs C++17
set_property(TARGET pose_graph_optimizer PROPERTY CXX_STANDARD 17)
set_property(TARGET pose_graph_optimizer PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(pose_graph_optimizer
        Eigen3::Eigen
        g2o::core
        g2o::stuff
        g2o::types_slam3d
        ${G2O_SOLVER_CSPARSE_LIB}
)

#---------------core lib------#

file(GLOB LIO_CORE_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/io/*.cpp
         ${SRC_PATH}/Align.cpp
         ${SRC_PATH}/common.cpp
         ${SRC_PATH}/lidar_odometry.cpp
         ${SRC_PATH}/ESKF.cpp
         ${SRC_PATH}/KeyFrame.cpp
         ${SRC_PATH}/RouteAlign.cpp
         ${SRC_PATH}/GPS.cpp
         ${SRC_PATH}/Lio/*.cpp
         ${SRC_PATH}/Slam/*.cpp
         ${SRC_PATH}/vis_utils.cpp
         ${SRC_PATH}/Submap.cpp
)
list(REMOVE_ITEM LIO_CORE_SOURCES  ${SRC_PATH}/Slam/Optimization.cpp)

add_library(lio_core ${LIO_CORE_SOURCES})
target_include_directories(lio_core PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lio_core PUBLIC
        rclcpp
        std_msgs
        rosbag2_cpp
        rosbag2_storage
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf2_msgs
)
target_link_libraries(lio_core PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX GDAL::GDAL yaml-cpp pose_graph_optimizer)

# Visualization node
add_executable(opengl_vis_node src/opengl_vis_node.cpp)
ament_target_dependencies(opengl_vis_node rclcpp std_msgs  geometry_msgs
        lio_msgs)
target_link_libraries(opengl_vis_node
        glad
        JPEG::JPEG
        OpenGL::GL
        ${GLFW_LIBRARIES}
        glm::glm
        Eigen3::Eigen
        lio_core
)


add_executable(mapping_node src/mapping_node.cpp)
ament_target_dependencies(mapping_node rclcpp std_msgs geometry_msgs
        lio_msgs)
target_link_libraries(mapping_node
        lio_core
)

add_executable(evaluate_system src/evaluate_system.cpp)
set_property(TARGET evaluate_system PROPERTY CXX_STANDARD 17)
target_link_libraries(evaluate_system lio_core)

#install(TARGETS
#        opengl_vis_node
# mapping_node
#        DESTINATION lib/${PROJECT_NAME}
#)

# ---- Install ----
install(TARGETS
        glad
        lio_core
        pose_graph_optimizer
        opengl_vis_node
        mapping_node
        evaluate_system
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION include)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lio_core test/test_io.cpp)
  target_link_libraries(test_lio_core lio_core Eigen3::Eigen)
  ament_add_gtest(test_shutdown test/test_shutdown.cpp)
  target_link_libraries(test_shutdown lio_core Eigen3::Eigen)
  ament_add_gtest(test_video_recorder test/test_video_recorder.cpp)
  target_link_libraries(test_video_recorder JPEG::JPEG)
  ament_add_gtest(test_intensity test/test_intensity.cpp)
  target_link_libraries(test_intensity lio_core Eigen3::Eigen)
  ament_add_gtest(test_evaluation test/test_evaluation.cpp)
  target_link_libraries(test_evaluation lio_core Eigen3::Eigen)
endif()


ament_package()
