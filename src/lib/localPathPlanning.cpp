#include "pathmaker/localPathPlanning.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace pm{

LocalPathPlanning::LocalPathPlanning(bool fromGzb)
    : fromGzb(fromGzb)
    , it_(ros::NodeHandle nh)
{
    //real wold msg
    if(!fromGzb){
        // 수정 필요
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
            &ImageConverter::imageCb, this);
    }
    //simulation msg
    else{
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
            &ImageConverter::imageCbForGzb, this);
    }

    //subscribe current position
    ros::Subscriber cur_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, curPoseCb);
}

void LocalPathPlanning::update()
{
    //depthMap을 통해 layer 만들고 경로점 찍는 것 까지.
    //layers.update(depthMap);
}

void LocalPathPlanning::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

void LocalPathPlanning::imageCbForGzb(const sensor_msgs::ImageConstPtr& msg)
{
    //send const msg
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //clone the converted image to depthMap and fit the format
    depthMap = cv_ptr->image.clone();
    depthMap.convertTo(depthMap, CV_16U, 1024.0)

    //layers update
    layers.update(depthMap);
}

void LocalPathPlanning::curPoseCb(const geometry_msgs::PoseStampedConstPtr& msg){
    curPose = *msg;
}

void LocalPathPlanning::publish(){
    local_pos_pub.publish(setPose);
}

Layers::Layers()
    : isGzb(false)
    , width_block(15)
    , height_block(9)
    , fov_h(87)
    , fov_v(58)
    , thereshold(5.0)
    , bin(height_block,width_block,CV_32F)
{

}

//in gazebo
Layers::Layers(bool isGzb)
    : isGzb(true)
    , width_block(15)
    , height_block(9)
    , fov_h(59)
    , fov_v(59*(480.0/848.0))
    , thereshold(5.0)
    , bin(height_block,width_block,CV_32F)
    , cost()
{

}

void update(const cv::Mat &depthMap){
    if(isGzb){
        for(int k=0;k<height_block;k++){
            for(int l=0;l<width_block;l++){
                for(int i=0;i<height_cut;i++){
                    for(int j=0;j<width_cut;j++)
                    {
                      depth_cal.at<float>(k,l)+=image.at<float>(i+1+k*height_cut,j+4+l*width_cut);
                      depth_cost.at<float>(k,l)+=18-image.at<float>(i+1+k*height_cut,j+4+l*width_cut);
                    }
                }
                if(depth_cal.at<float>(k,l)/(width_cut*height_cut) - margin_dist<0)
                {
                    binary_cal.at<uint8_t>(k,l)=1;
                }
                else
                    binary_cal.at<uint8_t>(k,l)=0;//평균으로 mat을 만들면 아주 작은물체가 가까이있으면 인식을 못할수도 있음
            }
        }///for
        //depth_cal=~depth_cal;
       depth = depth_cal/(width_cut*height_cut);
       depth_cost=depth_cost/(width_cut*height_cut);
       binary = binary_cal;
    }//callback
}

}