#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <cmath>

#include <iostream>

using namespace std;
quadrotor_msgs::PositionCommand pp;
ros::Publisher position_pub;
ros::Publisher takeoffland_pub;
ros::Subscriber odem_sub;
ros::Subscriber aim_sub;
bool started = false;

double preset_points[100][3]; // xyz
float nowpoint[3]; // xyz
bool detected = false;
double detected_point[3]; // xyz
bool droped = false;

void init_preset_points() {
    // first point
    preset_points[0][0] = 0;    // x
    preset_points[0][1] = 0;    // y
    preset_points[0][2] = 1.8;  // z

    // 4th point
    preset_points[1][0] = 0;
    preset_points[1][1] = -4.0;
    preset_points[1][2] = 1.8;

    // 5th point
    preset_points[2][0] = 0.8;
    preset_points[2][1] = -4.0;
    preset_points[2][2] = 1.8;

    // 8th point
    preset_points[3][0] = 0.8;
    preset_points[3][1] = 0;
    preset_points[3][2] = 1.8;

    // 9th point
    preset_points[4][0] = 1.6;
    preset_points[4][1] = 0;
    preset_points[4][2] = 1.8;

    // 12rd point
    preset_points[5][0] = 1.6;
    preset_points[5][1] = -4.0;
    preset_points[5][2] = 1.8;

    // 13rd point
    preset_points[6][0] = 2.4;
    preset_points[6][1] = -4.0;
    preset_points[6][2] = 1.8;

    // 16th point
    preset_points[7][0] = 2.4;
    preset_points[7][1] = 0;
    preset_points[7][2] = 1.8;

    // 17th point
    preset_points[8][0] = 3.2;
    preset_points[8][1] = 0;
    preset_points[8][2] = 1.8;

    // 20th point
    preset_points[9][0] = 3.2;
    preset_points[9][1] = -4.0;
    preset_points[9][2] = 1.8;

    // back point 1
    preset_points[10][0] = 1.6;
    preset_points[10][1] = -2.0;
    preset_points[10][2] = 1.8;

    // back point final
    preset_points[11][0] = -0.4;
    preset_points[11][1] = 0;
    preset_points[11][2] = 1.8;
}

double get_distance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2)); // z is not in use
}

bool drop_bomb() {
    droped = true;
}

void aim_pose_callback(geometry_msgs::PointStampedConstPtr pMsg) {
    detected = true;
    detected_point[0] = pMsg->point.x;
    detected_point[1] = pMsg->point.y;
    detected_point[2] = pMsg->point.z;
}

void start_callback(std_msgs::Int8 msg)
{
    started = true;
}

void odem_callback(nav_msgs::Odometry msg) {
    nowpoint[0] = msg.pose.pose.position.x;
    nowpoint[1] = msg.pose.pose.position.y;
    nowpoint[2] = msg.pose.pose.position.z;
    ROS_INFO("READ");
}

int main(int argc, char *argv[])
{
    init_preset_points();
    ros::init(argc, argv, "myplanner_node");
    ros::NodeHandle nh;
    odem_sub = nh.subscribe<nav_msgs::Odometry> (
        "/vins_fusion/imu_propagate",
        10,
        odem_callback
    );
    position_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd",
                                                                 10);
    takeoffland_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 
                                                                1);
    ros::Subscriber start_mission_sub = nh.subscribe<std_msgs::Int8>("start_mission", 1, start_callback);
    aim_sub = nh.subscribe<geometry_msgs::PointStamped>("aim_pose", 1, aim_pose_callback);
    ROS_INFO("my planner init!!!");

    ros::Rate r(10);
    
    quadrotor_msgs::TakeoffLand takeofflandmsg;
    
    // while (!started && ros::ok()) {
    //     ros::spinOnce();
    // }
    ros::Duration().fromSec(5).sleep(); // let man to leave
    ROS_INFO("mission_start!!!");
    takeofflandmsg.takeoff_land_cmd = 1; // this value mins takeoff
    takeoffland_pub.publish(takeofflandmsg);
    ros::Duration().fromSec(5).sleep();

    int this_point = 0; // mark the point to go 
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("target: %d", this_point);
        r.sleep();
        if (detected && !drop_bomb && false) { // always false
            pp.position.x = detected_point[0] + nowpoint[0];
            pp.position.y = detected_point[1] + nowpoint[1];
            pp.position.z = 1.0;
            if (get_distance(pp.position.x, pp.position.y, pp.position.z, nowpoint[0], nowpoint[1], nowpoint[2])< 0.4) {
                droped = drop_bomb();
            }
            position_pub.publish(pp);
            continue;
        } else {
            pp.position.x = preset_points[this_point][0];
            pp.position.y = preset_points[this_point][1];
            pp.position.z = preset_points[this_point][2];
            ROS_INFO("target x: %lf, y:%lf, z:%lf", pp.position.x, pp.position.y, pp.position.z);
            ROS_INFO("now_point x:%lf, y:%lf, z:%lf", nowpoint[0], nowpoint[1], nowpoint[2]);
            ROS_INFO("distance :%lf", get_distance(pp.position.x, pp.position.y, pp.position.z, nowpoint[0], nowpoint[1], nowpoint[2]));
            if (get_distance(pp.position.x, pp.position.y, pp.position.z, nowpoint[0], nowpoint[1], nowpoint[2])< 0.4) {
                this_point++;
                if (this_point >= 12) {
                    break;
                }
            }
            position_pub.publish(pp);
        }
    }
    ros::Duration().fromSec(3).sleep();
    takeofflandmsg.takeoff_land_cmd = 2; // this value mins land
    takeoffland_pub.publish(takeofflandmsg);
    return 0;
}
