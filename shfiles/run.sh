sudo chmod 777 /dev/ttyACM* & sleep 1;
roslaunch fdilink_ahrs ahrs_data.launch & sleep 5;
roslaunch realsense2_camera rs_camera.launch & sleep 5;
roslaunch mavros px4.launch & sleep 8;
rosrun mavros mavcmd long 511 105 2500 0 0 0 0 0 & sleep 3;
rosrun mavros mavcmd long 511 31 2500 0 0 0 0 0 & sleep 3;
roslaunch vins fast_drone_250.launch & sleep 2;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch ego_planner single_run_in_exp.launch & sleep 2;
roslaunch ego_planner rviz.launch & sleep 2;
wait;

