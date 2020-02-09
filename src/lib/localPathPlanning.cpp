#include "pathmaker/localPathPlanning.h"
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace pm{

LocalPathPlanning::LocalPathPlanning(ros::NodeHandle &nh, bool fromGzb)
    : fromGzb(fromGzb)
    , it_(nh)
    , originImgWindow(true)
    , check(false)
{
    //real wold msg
    if(!fromGzb){
        // 수정 필요
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
            &LocalPathPlanning::imageCb, this);
    }
    //simulation msg
    else{
        // Subscribe depth img
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
            &LocalPathPlanning::imageCbForGzb, this);
    }

    //subscribe current position
    ros::Subscriber cur_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, &LocalPathPlanning::curPoseCb, this);
}

void LocalPathPlanning::update()
{
    //depthMap을 통해 layer 만들고 경로점 찍는 것 까지.
    //layers.update(depthMap);
}

void LocalPathPlanning::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    this->depthMap = cv_ptr->image.clone();

    if(this->originImgWindow){
        cv::imshow(ORIGIN_IMG_WINDOW, this->depthMap);
        cv::waitKey(10);
    }
}

void LocalPathPlanning::imageCbForGzb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    //send const msg
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    
        //clone the converted image to depthMap and fit the format
        cv_ptr->image.convertTo(this->depthMap, CV_16U, 1024.0);
        //layers update
        // this->layers.update(depthMap);
        
        if(this->originImgWindow){
        cv::imshow(ORIGIN_IMG_WINDOW, this->depthMap);
        cv::waitKey(1);
    }    
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


}

void LocalPathPlanning::curPoseCb(const geometry_msgs::PoseStampedConstPtr& msg){
    curPose = *msg;
}


}