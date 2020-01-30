#ifndef __IMGPROC_H
#define __IMGPROC_H

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include "ImageConverter.h"
#include "pathmaker/PathRect.h"
#include "pathmaker/UAV.h"

class ImgProc{
public:
    //frame_size : horison * vertical (mm * mm)
    //cvtWindow : are you open the Converted image from msg to cv::Mat?
    //fromGzb : are you connect with gazebo?
    ImgProc(double rate, const UAV &uav, bool cvtWindow = false, bool fromGzb = false, bool prWindow = false);
    ~ImgProc();
private:
    void init();
    void run();
    void end();

    //
    cv::Mat img;
    
    ros::NodeHandle nh;
    ros::Publisher cmdToOffb;
    ros::Rate rate;

    //image convert to cv::Mat cv_ptr->image
    ImageConverter ict;

    PathRect pr;
    bool msgFromPR;
    // info of UAV
    const UAV &uav;
    
};

#endif