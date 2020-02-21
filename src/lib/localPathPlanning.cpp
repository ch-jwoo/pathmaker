#include "pathmaker/localPathPlanning.h"
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <kdl/utilities/utility.h>
// #include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
namespace pm{

LocalPathPlanning::LocalPathPlanning(ros::NodeHandle &nh, geometry_msgs::PoseStamped &curPose)
    : it_(nh)
    , curPose(curPose)
    , obstacleDetect(false)
{
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
        &LocalPathPlanning::imageCb, this);
    ROS_INFO("obtical ovidance ready");
}

void LocalPathPlanning::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    //send const msg
    try
    {
        if(msg->encoding == "32FC1"){
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depthMap = cv_ptr->image.clone();
            //clone the converted image to depthMap and fit the format
            // cv_ptr->image.convertTo(this->depthMap, CV_16U, 1024.0);
            //layers update
            // this->layers.update(depthMap);
            // ROS_INFO("middle depth : %f", depthMap.at<float>(depthMap.rows/2, depthMap.cols/2));
            
        }
        else if(msg->encoding == "16UC1"){
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_ptr->image.convertTo(this->depthMap, CV_32FC1);
        }
        else
        {
            ROS_ERROR("encoding inconsistent");
        }

        // cv::imshow(ORIGIN_IMG_WINDOW, this->depthMap);
        // cv::waitKey(1);

        layers.update(depthMap);
        calcTargetPose();
        // printf("azimuth, elevation : %f, %f, %d\n", layers.getAzimuth(), layers.getElevation(), obstacleDetect);
        if( layers.getAzimuth()<0.0698132 && layers.getAzimuth()>-0.0698132
            && layers.getElevation()<0.1598132 && layers.getElevation()>=0){
            obstacleDetect = false;
        }
        else{
            obstacleDetect = true;
        }
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


    // geometry_msgs::PoseStamped targetPose;//d*cos(elevation)*cos(azimuth),d*cos(elevation)*sin(azimuth),d*sin(elevation));
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


// void LocalPathPlanning::positionPub(){
//     static const float distance = 5.0;
//     float azimuth = layers.getAzimuth();
//     float elevation = layers.getElevation();

//     // tf::Transform transform;
//     // transform.setOrigin(tf::Vector3(curPose.position.x, curPose.position.y, curPose.position.z));
//     tf2::Quaternion q(curPose.pose.orientation.x, curPose.pose.orientation.y, curPose.pose.orientation.z, curPose.pose.orientation.w);
//     // transform.setRotation(q);
//     tfScalar curRoll, curPitch, curYaw;
//     tf2::Matrix3x3 temp(q);
//     temp.getRPY(curRoll, curPitch, curYaw);

//     float targetYaw = -azimuth+(float)curYaw;
//     q.setRPY(0, 0, targetYaw);


//     geometry_msgs::PoseStamped targetPose;//d*cos(elevation)*cos(azimuth),d*cos(elevation)*sin(azimuth),d*sin(elevation));
//     targetPose.pose.position.x = distance*KDL::cos(elevation)*KDL::cos(targetYaw)+curPose.pose.position.x;
//     targetPose.pose.position.y = distance*KDL::cos(elevation)*KDL::sin(targetYaw)+curPose.pose.position.y;
//     targetPose.pose.position.z = distance*KDL::sin(elevation)+curPose.pose.position.z;
//     targetPose.pose.orientation.x=q.x();
//     targetPose.pose.orientation.y=q.y();
//     targetPose.pose.orientation.z=q.z();
//     targetPose.pose.orientation.w=q.w();

//     // targetPose.pose.orientation(q.x, q.y, q.z, q.w);
//     // std::cout<< curPose.pose<<std::endl;
//     // std::cout << targetPose.pose << std::endl;
//     target_pos_pub.publish(targetPose);
// }

}