#ifndef __PATHPLANNING_H
#define __PATHPLANNING_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace pm{

class Layers;

class LocalPathPlanning{
private:
    Master &master;
    cv::Mat depthMap;
    Layers layers;

    //true : command to mavros_node
    bool check;

    //publish target position
    ros::Publisher local_pos_pub;
    geometry_msgs::PoseStamped setPose;
    
    //subscribe current position
    ros::Subscriber cur_pos_sub;
    geometry_msgs::PoseStamped curPose;

    //convert image msg to cv::Mat
    image_transport::ImageTransport it_
    image_transport::Subscriber image_sub_;

    //for send gzb msg
    bool fromGzb;
public:
    LocalPathPlanning(bool fromGzb = false);
    ~ImageConverter() = default;

    /**
     * @brief update check, pose
     **/
    void update();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void imageCbForGzb(const sensor_msgs::ImageConstPtr& msg);
    void curPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);
    void publish();
    inline void getCheck(){
        return check;
    }
};

class Layers{
private:
    cv::Mat bin;
    cv::Mat cost;
    cv::Mat dist_cost;
    const int width_block;
    const int height_block;
    const float fov_h;
    const float fov_v;
    const float threshold;

    bool isGzb;
public:
    //in real world
    Layers();

    //in gazebo
    Layers(bool isGzb);


    ~Layers() = default;
    void update(const cv::Mat &depthMap);
};


}



#endif