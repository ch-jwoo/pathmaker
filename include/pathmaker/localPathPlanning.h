#ifndef __PATHPLANNING_H
#define __PATHPLANNING_H
#include "layers.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace pm{

class LocalPathPlanning{
private:
    ros::NodeHandle nh;

    cv::Mat depthMap;
    Layers layers;

    //true : command to mavros_node
    bool check;

    //targetPose
    ros::Publisher target_pos_pub;
    geometry_msgs::PoseStamped setPose;
    
    //subscribe current position
    ros::Subscriber cur_pos_sub;
    geometry_msgs::PoseStamped curPose;

    //convert image msg to cv::Mat
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    ros::Time last_mode_time;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void curPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);

    //for send gzb msg
    bool fromGzb;

    //debug frame
    bool originImgWindow;
    const std::string ORIGIN_IMG_WINDOW = "Original image";

public:
    LocalPathPlanning(ros::NodeHandle &nh, bool fromGzb = false);
    ~LocalPathPlanning() = default;

    /**
     * @brief update check, pose
     **/
    void update();

    void positionPub();

    inline bool getCheck() const
    {
        return this->check;
    }

    inline geometry_msgs::PoseStamped getPose() const
    {
        return this->setPose;
    }
};




}



#endif