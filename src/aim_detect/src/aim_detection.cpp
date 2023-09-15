#include "aim_detect.h"

std_msgs::Int32 test_int;
ros::Publisher pic_pose_pub;

cv::Mat src;
// std_msgs::Float32MultiArray msg;
ros::Subscriber color_sub;
ros::Subscriber run_sub;

ros::Publisher id_pub;
std_msgs::Int32 aim_id;
geometry_msgs::PointStamped pic_pose;
void callback(Mat src)
{
#ifdef slip_debug

    namedWindow("red_mines_Trackbars", WINDOW_FREERATIO);

    // createTrackbar("red_c0_threshold", "red_mines_Trackbars", &red_c0_threshold, 255);
    createTrackbar("red_c1_threshold", "red_mines_Trackbars", &red_c1_threshold, 255);
    // createTrackbar("red_c2_threshold", "red_mines_Trackbars", &red_c2_threshold, 255);

#endif

    // if (detect_cnt > 0)
    // {
    //     detect_cnt--;
    //     if (detect_cnt == 0)
    //     {
    //         // ROS_INFO("goal_target: %d", goal_target);
    //         reach_goal = false;
    //     }
    // }

    waitKey(1);
    Mat _src;
    // resize(src, src, cv::Size(), 0.6, 0.6);
    src.copyTo(_src);

    GaussianBlur(_src, _src, Size(3, 3), 0, 0);

    std::vector<cv::Mat> split_channels;
    Point2f aim_pose = {-1, -1};

    ball_detect(src, _src, split_channels, aim_pose);
    if (aim_pose.x != -1)
    {
        pic_pose.point.x = aim_pose.x;
        pic_pose.point.y = aim_pose.y;
        pic_pose_pub.publish(pic_pose);
    }

#ifdef slip_debug

    imshow("red_mines_Trackbars", _src);
#endif

#ifdef show_final
    imshow("src", src);
#endif

#ifdef cut_img
    imshow("mask_roi", warp_mask);
    //
    imshow("src_roi", cut_src);

#endif

    //------------------------------------------------------------------------------
}

void color_callback(const std_msgs::Int32 color)
{
}

void run_callback(const std_msgs::Int32 run_key)
{
}

int main(int argc, char *argv[])
{

    // 如需压缩话题，则直接解除注释
    ros::init(argc, argv, "aim_detection");
    ros::NodeHandle nh;

    pic_pose_pub = nh.advertise<geometry_msgs::PointStamped>("pic_pose", 1);
    // run_sub = nh.subscribe("detect_over", 1, run_callback);

    ROS_INFO("aim detection init!!!\n");
    VideoCapture capture;
    Mat frame;
    capture.open("/dev/video6", CAP_V4L2);
    capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(CAP_PROP_FRAME_WIDTH, 1920);
    capture.set(CAP_PROP_FRAME_HEIGHT, 1080);
    capture.set(CAP_PROP_FPS, 60);
    if (!capture.isOpened())
    {
        ROS_INFO("fail to open cam!!!");
        return -1;
    }
    ROS_INFO("aim detect start");

    while (ros::ok())
    {

        capture >> frame;
        callback(frame);
    }

    ros::spin();
    return 0;
}
