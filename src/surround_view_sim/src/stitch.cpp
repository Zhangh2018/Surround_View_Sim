#include <ros/ros.h>
#include "Stitch.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core.hpp>
#include <vector>


ros::Publisher publisher;
std::vector<cv::Mat> inputs;
cv::Mat com_res;

void callbackStable(
        const sensor_msgs::ImageConstPtr& camera0_msg,
        const sensor_msgs::ImageConstPtr& camera1_msg,
        const sensor_msgs::ImageConstPtr& camera2_msg,
        const sensor_msgs::ImageConstPtr& camera3_msg)
{
    inputs[0] = cv_bridge::toCvShare(camera0_msg, "bgr8")->image;
    inputs[1] = cv_bridge::toCvShare(camera1_msg, "bgr8")->image;
    inputs[2] = cv_bridge::toCvShare(camera2_msg, "bgr8")->image;
    inputs[3] = cv_bridge::toCvShare(camera3_msg, "bgr8")->image;

    com_res = Stitch(inputs);

    cv_bridge::CvImage out_msg;
    out_msg.header   = camera0_msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = camera0_msg->encoding;
    out_msg.image    = com_res;

    publisher.publish(out_msg.toImageMsg());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stitch_node");
    ros::NodeHandle nh("~");
    
    std::string config_dir;
    nh.param<std::string>("config_dir", config_dir, "./modules/sw_stitch/data");    int InitStitch(std::string intrinsic_yml, std::string stitch_yml);
    InitStitch(config_dir + "/intrinsic.yml", config_dir + "/stitch.yml");

    publisher = nh.advertise<sensor_msgs::Image>("/sensor/camera/composed_image", 1);

    message_filters::Subscriber<sensor_msgs::Image> camera0(nh , "/sensor/camera_a00/image_raw", 10);//front
    message_filters::Subscriber<sensor_msgs::Image> camera1(nh , "/sensor/camera_a01/image_raw", 10);//right
    message_filters::Subscriber<sensor_msgs::Image> camera2(nh , "/sensor/camera_a02/image_raw", 10);//back
    message_filters::Subscriber<sensor_msgs::Image> camera3(nh , "/sensor/camera_a03/image_raw", 10);//left

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::Image
    > MySyncPolicy;

    inputs.resize(4);

    message_filters::Synchronizer< MySyncPolicy > sync( MySyncPolicy( 10 ), camera0, camera1, camera2,camera3);
    sync.registerCallback( boost::bind(callbackStable,_1, _2,_3,_4 ) );

    ros::Rate rate(50);

    while( ros::ok())
    {
        //printf("spinOnce\n");
        ros::spinOnce();
        rate.sleep();
    }

    ReleaseStitch();
    return 0;
}