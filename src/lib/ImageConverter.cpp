#include "pathmaker/ImageConverter.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ImageConverter::ImageConverter(const ros::NodeHandle &nh, cv::Mat &img,bool openWindow, bool fromGzb)
    : it_(nh), resultImg(img), openWindow(openWindow), fromGzb(fromGzb)
{
    if(!fromGzb){
        // 수정 필요
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
            &ImageConverter::imageCb, this);
    }
    else{
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
            &ImageConverter::imageCbForGzb, this);
    }
    if(openWindow){
        cv::namedWindow(WINDOW_NAME);
    }
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    resultImg = cv_ptr->image.clone();
    
    if(openWindow){
        cv::imshow(WINDOW_NAME, resultImg);
        cv::waitKey(10);
    }
}

void ImageConverter::imageCbForGzb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.convertTo(resultImg, CV_16U, 1024.0);
    
    if(openWindow && !(resultImg.empty())){
        cv::imshow(WINDOW_NAME, resultImg);
        cv::waitKey(10);
    }
}