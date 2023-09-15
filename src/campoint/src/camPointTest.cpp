#include <cstdio>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ros::Subscriber pointSub;
ros::Publisher aim_pose_pub;
ros::Subscriber z_pose_sub;;

float odom_z;

void pose_callback(nav_msgs::Odometry msg)
{
    odom_z = msg.pose.pose.position.z;
    // ROS_INFO("odom_z: %f", odom_z);
}

void detect(geometry_msgs::PointStamped msg)
{
    sensor_msgs::CameraInfo caminfo;
    caminfo.K.at(0) = 874.81358548;
    caminfo.K.at(2) = 949.37653351;
    caminfo.K.at(4) = 875.12612619;
    caminfo.K.at(5) = 570.35386798;

    geometry_msgs::PointStamped center_point;
    // 设置中心点的坐标系为 "cam3d"
    center_point.header.frame_id = "cam3d";
    center_point.header.stamp = ros::Time::now();
    // center_point.point.x = 10;
    // center_point.point.y = 5;
    // center_point.point.z = 10;

    try
    {
        // 获取圆环中心的像素坐标
        geometry_msgs::PointStamped point_pixel;
        point_pixel.header.frame_id = "cam3d";
        point_pixel.header.stamp = ros::Time::now();

        point_pixel.point.x = msg.point.x;
        point_pixel.point.y = msg.point.y;
        point_pixel.point.z = odom_z;

        // ROS_INFO("Oringin Point: x:%f  y:%f  z: 1", msg.point.x, msg.point.y);

        // tested
        // 通过相机内参，将圆环的二维深度图像坐标(像素坐标)转换为相机坐标系下的三维深度坐标
        center_point.point.z = point_pixel.point.z;
        center_point.point.y = -((point_pixel.point.x - caminfo.K.at(2)) / caminfo.K.at(0) * point_pixel.point.z);
        center_point.point.x = -((point_pixel.point.y - caminfo.K.at(5)) / caminfo.K.at(4) * point_pixel.point.z);

        // center_point.point.x = point_pixel.point.x;
        // center_point.point.y = -point_pixel.point.y;
        // center_point.point.z = -point_pixel.point.x;
        ROS_INFO("Receive Point: x:%f  y:%f  z:%f", center_point.point.x, center_point.point.y, center_point.point.z);

        aim_pose_pub.publish(center_point);
    }
    catch (cv::Exception &e)
    {
        ROS_WARN("%s\n", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "campoint"); // 初始化ros 节点，命名为 campoint
    ros::NodeHandle nh;                // 创建node控制句柄

    pointSub = nh.subscribe("pic_pose", 1, detect);
    aim_pose_pub = nh.advertise<geometry_msgs::PointStamped>("aim_pose", 1);
    z_pose_sub = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/odometry", 1, pose_callback);


    ros::Rate r(30);

    ros::spin();

    return 0;
}
