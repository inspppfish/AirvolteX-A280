// #include <cstdio>
// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <nav_msgs/Odometry.h>
// #include <opencv4/opencv2/highgui/highgui.hpp>
// #include <opencv4/opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <Eigen/Eigen>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/Vector3.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"


// void cam_point_cb(geometry_msgs::PoseStamped &msg){
//     ROS_INFO("Receive Point: x:  y:",msg.pose.position.x)
// }
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "campointSub"); // 初始化ros 节点，命名为 campoint
//     ros::NodeHandle hn;                   // 创建node控制句柄

//     ros::Subscriber pointSub = hn.subscribe("cam_fix_point", 1, cam_point_cb); // camerainfo

//     return 0;
// }