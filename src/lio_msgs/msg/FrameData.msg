std_msgs/Header header
uint64 frame_id
uint64 submap_id
uint64 loop_role


string cloud_path
geometry_msgs/Pose lidar_pose
geometry_msgs/Pose rtk_pose
geometry_msgs/Pose lidar_pose_neu
geometry_msgs/Pose fst_optimization_pose
geometry_msgs/Pose scd_optimization_pose
geometry_msgs/Pose loop_optimization_pose
geometry_msgs/Pose loop_ndt_pose

bool has_lidar_pose
bool has_rtk_pose
bool has_lidar_pose_neu
bool has_fst_optimization_pose
bool has_scd_optimization_pose
bool is_loop_closure
# Zero is ordinary mapping/replay; positive IDs identify inspection snapshots.
uint64 refresh_id
bool refresh_last
