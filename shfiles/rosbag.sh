rosbag record --tcpnodelay \
/mavros/imu/data_raw \
/vins_fusion/imu_propagate \
/vins_fusion/odometry \
/vins_fusion/path  \
/position_cmd  \
/debugPx4ctrl \
/drone_0_ego_planner_node/grid_map/occupancy \
/drone_0_odom_visualization/path
