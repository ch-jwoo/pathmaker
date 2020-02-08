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
#include<iostream>
static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 ros::Publisher chatter15 = nh_.advertise<std_msgs::Float32>("float32", 1000);
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/dep/lklk", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    std_msgs::Float32 msg15;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  cv::Mat &image = cv_ptr->image;
cv::Mat pixel=cv::Mat::zeros(6,8,CV_32F);
cv::Mat binary=cv::Mat::zeros(6,8,CV_32F);


for(int k=0;k<6;k++)
{
for(int l=0;l<8;l++)
{
for(int i=0;i<8;i++)
{
for(int j=0;j<8;j++)
{

pixel.at<float>(k,l)+=image.at<float>(i+k*8,j+l*8);
if(pixel.at<float>(k,l)-8*8*2<0)
{
binary.at<float>(k,l)=1;
}
else
binary.at<float>(k,l)=0;



}
}
}
}




//ROS_INFO("\n%f   %f     %f\n%f   %f     %f\n%f   %f     %f\n",pixel.at<float>(0,0),pixel.at<float>(0,1),pixel.at<float>(0,2),pixel.at<float>(1,0),pixel.at<float>(1,1),pixel.at<float>(1,2),pixel.at<float>(2,0),pixel.at<float>(2,1),pixel.at<float>(2,2));

std:: cout << "binary = \n" << binary << ";" << std::endl << std::endl;
 // std:: cout << "age = \n" << age << ";" << endl << endl;1
  std:: cout << "pixel = \n" << pixel << ";" << std::endl << std::endl;


/*
    int row=(image.rows);
    int col=(image.cols);
float depth=image.at<float>(col/2,row/2);    // Draw an example circle on the video stream
 ROS_INFO("%f",depth);
image_pub_.publish(cv_ptr->toImageMsg());
    msg15.data = depth;
    chatter15.publish(msg15);
*/
}
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bridge_test");
  ImageConverter ic;
  ros::spin();
  return 0;
}
