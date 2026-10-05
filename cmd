cd ~/Lidar3DSLAM_Localization
source /opt/ros/foxy/setup.bash
 colcon build --packages-select lio_msgs   --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/lio_msgs/share/lio_msgs/local_setup.bash
source install/setup.bash


  cd /home/chuchu/Lidar3DSLAM_Localization
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  ros2 run lio_viz opengl_vis_node