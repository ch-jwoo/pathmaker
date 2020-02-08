/* bridge_test.cpp
 * subscribe /camera/depth/image_raw
 * publisher /bridge_test/output_video
*/
#include <std_msgs/Float32.h> 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include "mavros_msgs/PositionTarget.h"
#include "nav_msgs/Odometry.h"
#include "cmath"
#include<iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include <mavros_msgs/VFR_HUD.h>



static const std::string OPENCV_WINDOW = "Image window";

int width_block = 15;
int height_block = 9;
float fov_h=59;
float fov_v=59*(480.0/848.0);
cv::Mat depth=cv::Mat::zeros(height_block,width_block,CV_32F);
cv::Mat depth_cost=cv::Mat::zeros(height_block,width_block,CV_32F);
cv::Mat binary=cv::Mat::zeros(height_block,width_block,CV_8UC1); 
mavros_msgs::PositionTarget local_pose_target;
geometry_msgs::PoseStamped local_pos;
mavros_msgs::WaypointList waypoint;
mavros_msgs::VFR_HUD instrument;
void local_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    local_pose_target = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    local_pos = *msg;
}

void waypoint_cb(const mavros_msgs::WaypointList::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    waypoint= *msg;
}

void instrument_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg)  /////mavros_msgs::(mavros msg type)::ConstPtr
{
    instrument= *msg;
}

struct EulerAngles {
    double roll, pitch, yaw;
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/depth/lklk", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  cv::Mat &image = cv_ptr->image;

    int width_block = 15;
    int height_block = 9;
    int width_cut =56;
    int height_cut=53;
    double margin_dist =5.0; 
    cv::Mat depth_cal=cv::Mat::zeros(height_block,width_block,CV_32FC1);
    cv::Mat binary_cal=cv::Mat::zeros(height_block,width_block,CV_8UC1); 

    for(int k=0;k<height_block;k++)
        {
            for(int l=0;l<width_block;l++)
            {
                for(int i=0;i<height_cut;i++)
                {
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
                binary_cal.at<uint8_t>(k,l)=0;                                       /////////////////평균으로 mat을 만들면 아주 작은물체가 가까이있으면 인식을 못할수도 있음
            }
        }///for
        //depth_cal=~depth_cal;
       depth = depth_cal/(width_cut*height_cut);
       depth_cost=depth_cost/(width_cut*height_cut);
       binary = binary_cal;
    }//callback

};//class




int main(int argc, char** argv)
{
  ros::init(argc, argv, "bridge_test");
  ros::NodeHandle nh;
  ros::Subscriber local_target_sub = nh.subscribe<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/target_local", 10,local_pos_target_cb);
  ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10,local_pos_cb);
  ros::Subscriber mission_waypoint = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10,waypoint_cb);
  ros::Subscriber climb = nh.subscribe<mavros_msgs::VFR_HUD>
            ("mavros/vfr_hud", 10,instrument_cb);
          

  ros::Time::init();
  ros::Rate loop_rate(10);
  ImageConverter ic;
  double minVal;
  double maxVal;
  
  cv::Point minPixel;
  cv::Point maxPixel;
  EulerAngles angles;
  
  /*
  double sinr_cosp = 2 * (q.getW * q.getX + q.getY * q.getZ);
  double cosr_cosp = 1 - 2 * (q.getX * q.getX + q.getY * q.getY);
  angles.roll = atan2f64(sinr_cosp, cosr_cosp);
  
  double sinp = 2 * (q.getW * q.getY - q.getZ * q.getX);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp);
    else
        angles.pitch = std::asin(sinp);
  double siny_cosp = 2 * (q.getW * q.getZ + q.getX* q.getY);
    double cosy_cosp = 1 - 2 * (q.getY * q.getY + q.getZ * q.getZ);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
*/


  while(ros::ok())
  {
    std:: cout << "binary = \n" << binary << ";" << std::endl << std::endl;
    std:: cout << "depth = \n" << depth_cost << ";" << std::endl << std::endl;
    
                 ///////////////masking layer CV_8UC1으로 해야댐
    

    ros::Time last_request = ros::Time::now();
    //float lat_x =(float)waypoint.waypoints[1].x_lat;
    //ROS_INFO("target pos x %f", lat_x);
    //_Float64 position_X = (_Float64)waypoint.waypoints[1].x_lat;
    //std:: cout << "depth = \n" << position_X << std::endl << std::endl;
    tf::Quaternion q (local_pos.pose.orientation.x,local_pos.pose.orientation.y,local_pos.pose.orientation.z,local_pos.pose.orientation.w);
    tf::Matrix3x3 m(q);
    tfScalar current_roll,current_pitch,current_yaw;
    m.getRPY(current_roll,current_pitch,current_yaw,1);                             ///////////// 마지막에 1 넣어줘야댐(sovlver 설정)
    /*
    printf("QX= %.6f \n",local_pos.pose.orientation.x);
    printf("Qy= %.6f \n",local_pos.pose.orientation.y);
    printf("Qz= %.6f \n",local_pos.pose.position.z);
    printf("Climb rate= %.6f \n",instrument.climb);
    printf("Ground speed= %.6f \n",instrument.groundspeed);
    printf("Qw= %.6f \n",local_pos.pose.orientation.w);
    printf("R= %.6f \n",current_pitch);                 //*3.14159265*57.2957795786
    printf("P= %.6f \n",current_roll);
    printf("Y= %.6f \n",current_yaw);
    printf("Target Y= %.6f \n",local_pose_target.yaw);
    */

    //std:: cout << local_pose_target.position << ";\n" << local_pos.pose.position << "\n" << std::endl << std::endl;
    //float dist=sqrtf64(pow(local_pose_target.position.x-local_pos.pose.position.x,2.0)+pow(local_pose_target.position.y-local_pos.pose.position.y,2.0));
    //double del_theta=acosf64((local_pose_target.position.x-local_pos.pose.position.x)/dist);
    //double del_elev= atanf64((local_pose_target.position.z-local_pos.pose.position.z)/dist);
    float del_theta = (local_pose_target.yaw-current_yaw)*57.2957795786;
    float del_elev = atanf(instrument.climb/instrument.groundspeed)*57.2957795786;
    cv::Mat cost_mat=cv::Mat::zeros(height_block,width_block,CV_32FC1);
    
    //float Kd =1.0;
    float Kh =0.7;
    float Ke =0.4;


    for(int i=0;i<height_block;i++){   ///i 0~7 8칸
      for(int j=0;j<width_block;j++){   ///j 0~13 14칸
          
          cost_mat.at<float>(i,j)=/*Kd*/depth_cost.at<float>(i,j)+Kh*abs((del_theta+fov_h/2.0)-(fov_h/width_block)*(j+1/2.0))+Ke*abs((del_elev+fov_v/2.0)-(fov_v/height_block)*(((height_block-1)-i)+1/2.0));
          
          
          /*if(del_elev<-30){
            cost_mat.at<float>(i,height_block-1)=Kd*depth_cost.at<float>(i,j)+Kh*abs((del_theta+fov_h/2.0)-(fov_h/width_block)*(j+1/2.0));
          }
          else if(del_elev>30){
            cost_mat.at<float>(i,j)=Kd*depth_cost.at<float>(i,j)+Kh*abs((del_theta+fov_h/2.0)-(fov_h/width_block)*(j+1/2.0))+100.0;
          }
          else
            cost_mat.at<float>(i,j)=Kd*depth_cost.at<float>(i,j)+Kh*abs((del_theta+fov_h/2.0)-(fov_h/width_block)*(j+1/2.0))+Ke*abs((del_elev+fov_v/2.0)-(fov_v/height_block)*(((height_block-1)-i)+1/2.0));
          */
            
        }
      }
    /*
    if(del_elev<-30){
      cv::minMaxLoc(cost_mat(cv::Range(0,0),cv::Range(0,13)),&minVal,&maxVal,&minPixel,&maxPixel,~binary);
    }
    else if(del_elev>30){
      cv::minMaxLoc(cost_mat(cv::Range(7,0),cv::Range(7,13)),&minVal,&maxVal,&minPixel,&maxPixel,~binary);
    }
    else
      cv::minMaxLoc(cost_mat,&minVal,&maxVal,&minPixel,&maxPixel,~binary);
    */
    printf("del_theta= %.6f \n",del_theta);
    printf("del_elev= %.6f \n",del_elev);
    std::cout << binary << std::endl << std::endl;
    std::cout << cost_mat << std::endl << std::endl;
    cv::minMaxLoc(cost_mat,&minVal,&maxVal,&minPixel,&maxPixel,~binary);
    std:: cout << "minmax = \n" << minVal <<";" << minPixel <<";" << std::endl << std::endl;


    
    //std:: cout << dist << "\n" << del_elev*57.2957795786 << "\n" << del_theta*57.2957795786 << "\n" << std::endl;
    
    //printf("dist = %.6f\n",dist);
    //printf("elev= %.6f \n",del_elev*57.2957795786);
    

    

    loop_rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}