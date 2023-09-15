#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

using namespace std;

static geometry_msgs::PoseStamped *  generate_single_pose(int id) {
    auto p = new geometry_msgs::PoseStamped();

    p->header.frame_id = "map";
    p->header.stamp = ros::Time::now();

    p->pose.orientation.w = 1;
    p->pose.orientation.x = 0.0;
    p->pose.orientation.y = 0.0;
    p->pose.orientation.z = 0.0;

    if (id == 0) {
        p->pose.position.x = 1;
        p->pose.position.y = 0;
        p->pose.position.z = 1;
    } else if (id == 1) {
        p->pose.position.x = 0;
        p->pose.position.y = 0;
        p->pose.position.z = 1;
    } else {
        p = nullptr;
    }
    return p;
}

static geometry_msgs::PoseStamped * generate_next_pose(void) {
    static int id = 0;
    auto p = generate_single_pose(id);
    ROS_INFO("generated pose %d\n", id);
    id++;
    return p;
}



int main (int argc, char** argv) {
    // normal init
    ros::init(argc, argv, "mydemo_node");
    ros::NodeHandle nh("~");

    // publisher
    auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mywaypiontpose", 1);

    //simple ros loop with delay
    while (ros::ok()) {
        auto pose_ = generate_next_pose();
        if (pose_) {
            pose_pub.publish(*pose_);
            delete pose_;
        }
        ros::Duration(8.0).sleep();
    }

    return 0;
}