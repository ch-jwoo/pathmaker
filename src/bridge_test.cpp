#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

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
    image_pub_ = it_.advertise("/bridge_test/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat &refImg = cv_ptr->image;
    cv::Mat img;
    refImg.convertTo(img, CV_16U, 1024.0);
    
    // Draw an example circle on the video stream
    if (img.rows > 60 && img.cols > 60)
      cv::circle(img, cv::Point(img.rows/2, img.cols/2), 50, CV_RGB(255,255,255),-1);

    ROS_INFO("depth : %d", img.depth());
    ROS_INFO("depth : (%d)", img.at<uint16_t>(img.rows/2, img.cols/2));
    
    // Update GUI Window
    //cv::resize((cv_ptr->image), (cv_ptr->image), cv::Size(640,480));
    
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(10);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bridge_test");
  ImageConverter ic;
  ros::spin();
  return 0;
}