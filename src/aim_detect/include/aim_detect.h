// ros
#include <ros/ros.h>
#include <ros/time.h>
// #include <camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PointStamped.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

// #define debug
// #define slip_debug
#define show_final
// #define show_src

int run_aim_detect = 0;
int our_color = 2; /// 我方：1是蓝，2是红
int goal_target;   /// 0:none , 1:bluereal, 2:redreal, 3:bluefake, 4:redfake
int detect_cnt = 0;
const int threshold_detect_cnt = 5;
const int max_detect_cnt = 60;
bool reach_goal = false;
const float max_dist = 150;
const float area_rate = 0.73; 

int red_c1_threshold = 178;


const int min_area = 20;
const int max_area_2 = 1500;



bool shape_detect(Mat src, vector<vector<Point>> find_contours, Point2f& aim_pose)
{
    // ROS_INFO("shape_detect!!!");
    vector<Point> sim_approx;
    Point2f temp_pose;
    for (size_t i = 0; i < find_contours.size(); i++)
    {
        double area = contourArea(find_contours[i]);
        Point2f center;
        float radius;
        minEnclosingCircle(find_contours[i], temp_pose, radius);
        double min_cirle_area = 3.14 * radius * radius;
        if (area / min_cirle_area > area_rate)
        {
#ifdef show_final
            aim_pose = temp_pose;
            circle(src, aim_pose, 4, Scalar(0, 0, 255), -2);
            // ROS_INFO("fianl_area: %f", area);
            
            return true;

#endif
        }

    }
    return false;
}

bool ball_detect(Mat &src, Mat &mask, std::vector<cv::Mat> &channels, Point2f& aim_pose)
{
    // ROS_INFO("ball_detect!!!");
    // cvtColor(mask, mask, COLOR_BGR2GRAY); // 将图像转为灰度图

    cv::split(mask, channels);
    // Mat real_templateImage, fack_templateImage;

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3)); // 定义卷积核

    // cv::threshold(channels[0], channels[0], red_c0_threshold, 255, cv::THRESH_BINARY);
    // cv::threshold(channels[2], channels[2], red_c2_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(channels[1], channels[1], red_c1_threshold, 255, cv::THRESH_BINARY);
    // cv::threshold(mask, mask, red_c1_threshold, 255, cv::THRESH_BINARY);

    mask = channels[1];


    // imshow("c1", channels[1]);

    // imshow("c2", channels[2]);
    erode(mask, mask, kernel);
    dilate(mask, mask, kernel);

    vector<vector<Point>> contours;
    vector<vector<Point>> find_contours;

    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // 查找轮廓
    int max_area = INT_MIN;
    for (size_t i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) > min_area && contourArea(contours[i]) < max_area_2)
        {

            // ROS_INFO("AREA : %f", contourArea(contours[i]));
            find_contours.push_back(contours[i]);
            // drawContours(src, contours, i, Scalar(255, 0, 0), 2);
            // ROS_INFO("find contours over\n");
        }
    }

    if (find_contours.empty())
    {
        // ROS_INFO("find contours fail!!!\n");
        waitKey(1);
        return false;
    }

    vector<Point> find_contour;

    vector<Point> find_approx;
    if (!shape_detect(src, find_contours, aim_pose))
    {
#ifdef slip_debug

        // ROS_INFO("gogogo");
#endif
        return false;
    }

    putText(src,"(" + to_string(aim_pose.x) + ", " + to_string(aim_pose.y) + ")", aim_pose, FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 1.5);
    return true;
}