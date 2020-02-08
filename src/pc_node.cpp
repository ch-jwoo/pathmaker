#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

geometry_msgs::PoseStamped cur_pose;
PointCloud::Ptr cloud;

void subLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    cur_pose = *msg;
}
void subPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg){
    Eigen::Affine3f angle;
    Eigen::AngleAxisf fa;
    fa.
    angle.rotation()
    
    cloud = PointCloud::Ptr(new PointCloud);
    
}
void printPose(const geometry_msgs::PoseStamped& pose){
    printf("position (x, y, z) = (%f, %f, %f)\n",
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    printf("quaternion (x, y, z, w) = (%f, %f, %f, %f)\n",
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pc_node");
    ros::NodeHandle nh;

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, subLocalPose);
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>
            ("camera/depth/points", 10, subPointCloud);
    
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_publish",50);


    ros::Rate rate(20.0);

    while(ros::ok()){
        printPose(cur_pose);
        point_cloud_pub.publish(cloud);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}