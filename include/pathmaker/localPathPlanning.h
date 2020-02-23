#ifndef __PATHPLANNING_H
#define __PATHPLANNING_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "layers.h"

namespace pm{

class LocalPathPlanning{
private:
    geometry_msgs::PoseStamped &curPose;

    cv::Mat depthMap;
    Layers layers;

    bool obstacleDetect;
    
    //targetPose
    geometry_msgs::PoseStamped targetPose;
    
    //convert image msg to cv::Mat
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void calcTargetPose();
public:
    LocalPathPlanning(ros::NodeHandle &nh, geometry_msgs::PoseStamped &curPose);
    ~LocalPathPlanning() = default;

    /**
     * @brief update check, pose
     **/
    inline geometry_msgs::PoseStamped& getTargetPose(){
        return targetPose;
    }

    inline bool obstacleDetected(){
        return obstacleDetect;
    }
};




}



#endif