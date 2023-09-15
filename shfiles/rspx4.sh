#!/bin/bash
sudo chmod 777 /dev/ttyACM0 & sleep 1;
source /home/uava/Fast-Drone-250_230708/devel/setup.bash
roslaunch fdilink_ahrs ahrs_data.launch & sleep 3;
roslaunch realsense2_camera rs_camera.launch & sleep 2;
roslaunch mavros px4.launch & sleep 2;
rosrun mavros mavcmd long 511 105 2500 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 31 2500 0 0 0 0 0 & sleep 2;
roslaunch vins fast_drone_250.launch
wait;
