#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <std_msgs/String.h>
#include <cstdlib>
#include <mavros_msgs/Waypoint.h>
#include <iostream>
#include <math.h>
#define MIS 3
mavros_msgs::HomePosition gp;
void get_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
  //ros::Duration(3).sleep();
  gp=*msg;


}


int main(int argc, char **argv)
{
  //get_cb;
  _Float32 dest[2];
  _Float32 waypoints[6];
  _Float32 lon,lat,alt;
  _Float32 diff;
  std::cout<<"dest";
  std::cin >> dest[0];//x0
  std::cin >> dest[1];//y0

  ros::init(argc, argv, "pushing_waypoint");
  //ros::NodeHandle p;
  ros::NodeHandle n;
  //ros::NodeHandle l;
  ros::ServiceClient wp_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear"); //("waypoint_clear_client")
  ros::ServiceClient wp_srv_client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
  ros::ServiceClient set_home_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
  ros::Subscriber  sub=n.subscribe ("/mavros/home_position/home",1, get_cb);
  ros::Rate rate(20.0);
  mavros_msgs::WaypointPush wp_push_srv;
  mavros_msgs::CommandHome set_home_srv;
  mavros_msgs::WaypointClear wp_clear_srv;
  
  set_home_srv.request.current_gps = true;
  //set_home_srv.request.altitude=535.0;
  //ros::Duration(3).sleep();
  //lon=gp.geo.longitude;
  //lat=gp.geo.latitude;
  
  while(ros::ok())
{
  lon=gp.geo.longitude;
  alt=0;//gp.geo.altitude;
  lat=gp.geo.latitude;
  ros::spinOnce();
  rate.sleep();
  if(lon!=0)
  {
    break;
  }
}

  //ros::Duration(3).sleep();

 //set_home_srv.request.latitude=47.3976941;
  //set_home_srv.request.longitude=8.5455172;
  //set_home_srv.request.altitude=0.0;


  std::cout.precision(10);


  /*
  lon=hp_.geo.longitude;//x3
  lat=hp_msg.geo.latitude;//y3
*/  waypoints[4]=lon+(dest[0]-lon)/30.0;//x0+(x3-x0)/3
  waypoints[0]=lon+(dest[0]-lon)/3.0;//x0+(x3-x0)/3
  waypoints[2]=lon+2.0*(dest[0]-lon)/3.0;//x0+2*(3-x0)/3  

  diff=(dest[1]-lat)/(dest[0]-lon);

  waypoints[1]=diff*(waypoints[0]-lon)+lat;
  waypoints[3]=diff*(waypoints[2]-lon)+lat;
  waypoints[5]=diff*(waypoints[4]-lon)+lat;
  std::cout<<"\nwaypoints1 lon=  "<<waypoints[0];
  std::cout<<"\nwaypoints1 lat=  "<<waypoints[1];
  std::cout<<"\nwaypoints2 lon=  "<<waypoints[2];
  std::cout<<"\nwaypoints2 lat=  "<<waypoints[3]<<"\n";
  mavros_msgs::Waypoint wp_msg;

  wp_clear_srv.request = {};
  wp_msg.frame = MIS; // mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp_msg.command = 16; //nav to waypo
  //wp_msg.is_current = true;
  wp_msg.autocontinue = true;
  wp_msg.param1 = 0;
  //wp_msg.param2 = 0;
  wp_msg.param3 = 0;
  wp_msg.param4 = NAN;
  wp_msg.x_lat = waypoints[5];
  wp_msg.y_long = waypoints[4];
  wp_msg.z_alt = alt+10.0;
  wp_push_srv.request.waypoints.push_back(wp_msg);

  wp_msg.frame = MIS; // mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp_msg.command = 16; //nav to waypo
  //wp_msg.is_current = true;
  wp_msg.autocontinue = true;
  wp_msg.param1 = 0;
  //wp_msg.param2 = 0;
  wp_msg.param3 = 0;
  wp_msg.param4 = NAN;
  wp_msg.x_lat = waypoints[1];
  wp_msg.y_long = waypoints[0];
  wp_msg.z_alt = alt+30.0;
  wp_push_srv.request.waypoints.push_back(wp_msg);

  wp_msg.frame = MIS; // mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp_msg.command = 16; //nav to waypo
  //wp_msg.is_current = true;
  wp_msg.autocontinue = true;
  wp_msg.param1 = 0;
  //wp_msg.param2 = 0;
  wp_msg.param3 = 0;
  wp_msg.param4 = NAN;
  wp_msg.x_lat = waypoints[3];
  wp_msg.y_long = waypoints[2];
  wp_msg.z_alt = alt+50.0;
  wp_push_srv.request.waypoints.push_back(wp_msg);
 
  wp_msg.frame = MIS; // mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp_msg.command = 21; //land
  //wp_msg.is_current = true;
  wp_msg.autocontinue = true;
  wp_msg.param1 = 0;
  wp_msg.param2 = 0;
  wp_msg.param3 = 0;
  wp_msg.param4 = NAN;
  wp_msg.x_lat = dest[1];
  wp_msg.y_long = dest[0];
  wp_msg.z_alt = alt+10.0;
  wp_push_srv.request.waypoints.push_back(wp_msg);


  //wp_clear_srv.request = {};

  //wp_push_srv.request.start_index = 0;

  //set_home_srv.request.current_gps = true;

  if (set_home_client.call(set_home_srv))
{
    ROS_INFO("Home was set to new value ");

}
else
{
    ROS_ERROR("Home position couldn't been changed");
}


if (wp_clear_client.call(wp_clear_srv))
{
    ROS_INFO("Waypoint list was cleared");
}
else
{
    ROS_ERROR("Waypoint list couldn't been cleared");
}

  if (wp_srv_client.call(wp_push_srv))
  {
    ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
  }
  else
  {
    ROS_ERROR("Waypoint couldn't been sent");
    ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
  }

  return 0;
}
