#ifndef __PATHRECT_H
#define __PATHRECT_H

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "pathmaker/UAV.h"
#include <thread>
class PathRect{
private:
    //copy ImgProc's img for edit
    const UAV &uav;

    cv::Mat img;
    cv::Mat imgROI;
    cv::Rect2i pathRect;

    double avgDepth;
    double minDepth;
    double maxDepth;
    cv::Point2i minLoc;
    cv::Point2i maxLoc;
    
    bool openWindow;
    const std::string WINDOW_NAME = "PathRect window";

    void calRect();
    void calAvg();
public:
    PathRect(const cv::Mat &img, const UAV &uav, bool openWindow);
    void updateROI(const cv::Mat &img);
};

#endif