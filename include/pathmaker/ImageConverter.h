#ifndef __IMAGECONVERTER_H
#define __IMAGECONVERTER_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat &resultImg;
    cv_bridge::CvImageConstPtr cv_ptr;
    const std::string WINDOW_NAME = "ImageConverter window";
    bool openWindow;
    bool fromGzb;
public:
    ImageConverter(const ros::NodeHandle &nh, cv::Mat &img, bool openWindow = false, bool fromGazebo = false);
    ImageConverter() = default;
    ~ImageConverter() = default;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void imageCbForGzb(const sensor_msgs::ImageConstPtr& msg);
    
};

#endif