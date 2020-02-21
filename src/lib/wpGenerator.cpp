#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include "pathmaker/wpGenerator.h"

#define EXALT 3
namespace pm{

WpGenerator::WpGenerator(ros::NodeHandle &node_handle)
    : nh(node_handle)
{
    wp_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("/mavros/mission/clear");
    wp_srv_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("/mavros/mission/push");
    set_home_client = nh.serviceClient<mavros_msgs::CommandHome>
            ("/mavros/cmd/set_home");
    home_pos_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("/mavros/home_position/home",1, &WpGenerator::homePosCb, this);
}

void WpGenerator::calSubWP()
{
    _Float64 lat_diff = target_lat - home_lat;
    _Float64 lon_diff = target_lon - home_lon;
    addWP(home_lat, home_lon, EXALT);
    addWP(home_lat + lat_diff/3.0, home_lon + lon_diff/3.0, EXALT);
    addWP(home_lat + lat_diff*2.0/3.0, home_lon + lon_diff*2.0/3.0, EXALT);
    addLand(target_lat, target_lon, 3);
}

void WpGenerator::homePosCb(const mavros_msgs::HomePositionConstPtr& msg)
{
    this->home_lat = msg->geo.latitude;
    this->home_lon = msg->geo.longitude;
}


void WpGenerator::addWP(_Float64 lat, _Float64 lon, _Float64 alt)
{
    mavros_msgs::Waypoint wp_msg;
    wp_msg.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT; // mavros_msgs::Waypoint::FRAME_GLOBAL;
    wp_msg.command = mavros_msgs::CommandCode::NAV_WAYPOINT; //waypoint
    //wp_msg.is_current = true;
    wp_msg.autocontinue = true;

    /**
     * @brief NAV_WAYPOINT param
     * 1: Hold Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)	min:0	s
     * 2: Accept Radius	Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)	min:0	m
     * 3: Pass Radius 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.		m
     * 4: Yaw Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).		deg
     * 5: Latitude		
     * 6: Longitude		
     * 7: Altitude
     **/
    wp_msg.param1 = 0;
    //wp_msg.param2 = 0;
    wp_msg.param3 = 0;
    wp_msg.param4 = NAN;
    wp_msg.x_lat = lat;
    wp_msg.y_long = lon;
    wp_msg.z_alt = alt;
    wp_push_srv.request.waypoints.push_back(wp_msg);
}

void WpGenerator::addLand(_Float64 lat, _Float64 lon, _Float64 alt)
{
    mavros_msgs::Waypoint wp_msg;
    wp_msg.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT; // mavros_msgs::Waypoint::FRAME_GLOBAL;
    wp_msg.command = mavros_msgs::CommandCode::NAV_LAND; //land
    //wp_msg.is_current = true;
    wp_msg.autocontinue = true;

    /**
     * 1: Abort Alt	Minimum target altitude if landing is aborted (0 = undefined/use system default).		m
     * 2: Land Mode	Precision land mode.	PRECISION_LAND_MODE	
     * 3: Empty.		
     * 4: Yaw Angle	Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).		deg
     * 5: Latitude.		
     * 6: Longitude.		
     * 7: Landing altitude (ground level in current frame).		m
     **/
    wp_msg.param1 = 0;
    wp_msg.param2 = 0;
    wp_msg.param3 = 0;
    wp_msg.param4 = NAN;
    wp_msg.x_lat = lat;
    wp_msg.y_long = lon;
    wp_msg.z_alt = alt;
    wp_push_srv.request.waypoints.push_back(wp_msg);
}

bool WpGenerator::pushWP()
{
    if (wp_srv_client.call(wp_push_srv) && wp_push_srv.response.success)
    {
        ROS_INFO("Waypoint push success");
    }
    else
    {
        ROS_ERROR("Waypoint couldn't been sent");
    }
    return wp_push_srv.response.success;
}

bool WpGenerator::current2Home()
{
    set_home_srv.request.current_gps = true;
    if (set_home_client.call(set_home_srv) && set_home_srv.response.success)
    {
        ROS_INFO("Home was set to new value ");
    }
    else
    {
        ROS_ERROR("Home position couldn't been changed");
    }
    return set_home_srv.response.success;
}

bool WpGenerator::cleanWP()
{
    if (wp_clear_client.call(wp_clear_srv) && wp_clear_srv.response.success)
    {
        ROS_INFO("Waypoint list was cleared");
    }
    else
    {
        ROS_ERROR("Waypoint list couldn't been cleared");
    }
    return wp_clear_srv.response.success;
}

bool WpGenerator::detectTarget(){
    mavros_msgs::WaypointListConstPtr cwp = ros::topic::waitForMessage<mavros_msgs::WaypointList>("/mavros/mission/waypoints");
    mavros_msgs::WaypointList wp = *cwp;
    std::cout<<wp.waypoints.size()<<std::endl;
    if(wp.waypoints.size() == 1){
        setTarget(wp.waypoints[0].x_lat, wp.waypoints[0].y_long);
        printf("%f, %f\n", wp.waypoints[0].x_lat, wp.waypoints[0].y_long);
        return true;
    }
    return false;
}

void WpGenerator::wpCb(const mavros_msgs::WaypointListConstPtr& msg){
    cur_wp = *msg;
}

}