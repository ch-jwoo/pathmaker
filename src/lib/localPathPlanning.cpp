#include "pathmaker/localPathPlanning.h"
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <kdl/utilities/utility.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
namespace pm{

LocalPathPlanning::LocalPathPlanning(ros::NodeHandle &nh, geometry_msgs::PoseStamped &curPose)
    : it_(nh)
    , curPose(curPose)
    , obstacleDetect(false)
{
    while(!ros::topic::waitForMessage<sensor_msgs::Image>
                ("/camera/depth/image_rect_raw", ros::Duration(5.0))){
        ROS_ERROR("There is no \"/camera/depth/image_rect_raw\" message");
    }

    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
        &LocalPathPlanning::imageCb, this);

    image_pub_ = it_.advertise("/image_from_lp", 1);
}

void LocalPathPlanning::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // cv_bridge::CvImageConstPtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr;
    //send const msg
    try
    {
        if(msg->encoding == "32FC1"){
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depthMap = cv_ptr->image.clone();
            
        }
        else if(msg->encoding == "16UC1"){
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_ptr->image.convertTo(this->depthMap, CV_32FC1);
        }
        else
        {
            ROS_ERROR("encoding inconsistent");
        }
        layers.update(depthMap);
        calcTargetPose();

        // cv::imshow(ORIGIN_IMG_WINDOW, this->depthMap);
        // cv::waitKey(1);

        // printf("azimuth, elevation : %f, %f, %d\n", layers.getAzimuth(), layers.getElevation(), obstacleDetect);
        if( layers.getAzimuth()<0.0698132 && layers.getAzimuth()>-0.0698132
            && layers.getElevation()<0.1598132 && layers.getElevation()>=0){
            obstacleDetect = false;
        }
        else{
            obstacleDetect = true;
        }
        
        //for debug
        depthMap.convertTo(cv_ptr->image, CV_16UC1);
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void LocalPathPlanning::calcTargetPose()
{
    static const float distance = 5.0;
    float azimuth = layers.getAzimuth();
    float elevation = layers.getElevation();

    tf2::Quaternion q(curPose.pose.orientation.x
                    , curPose.pose.orientation.y
                    , curPose.pose.orientation.z
                    , curPose.pose.orientation.w);

    tf2Scalar curRoll, curPitch, curYaw;
    tf2::Matrix3x3 temp(q);
    temp.getRPY(curRoll, curPitch, curYaw);

    float targetYaw = -azimuth+(float)curYaw;
    q.setRPY(0, 0, targetYaw);

    targetPose.pose.position.x = distance*KDL::cos(elevation)*KDL::cos(targetYaw)+curPose.pose.position.x;
    targetPose.pose.position.y = distance*KDL::cos(elevation)*KDL::sin(targetYaw)+curPose.pose.position.y;
    targetPose.pose.position.z = distance*KDL::sin(elevation)+curPose.pose.position.z;
    targetPose.pose.orientation.x=q.x();
    targetPose.pose.orientation.y=q.y();
    targetPose.pose.orientation.z=q.z();
    targetPose.pose.orientation.w=q.w();
    

    // std::cout <<"cur = "<< curPose << std::endl;
    // std::cout <<"target = "<< targetPose << std::endl;
}


}
