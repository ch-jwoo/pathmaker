#ifndef __WPGENERATOR_H
#define __WPGENERATOR_H

#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/HomePosition.h>

namespace pm{
//make sub waypoint from destination GPS
class WpGenerator{
private:
    ros::NodeHandle &nh;

    //for wp clear
    ros::ServiceClient wp_clear_client;
    mavros_msgs::WaypointClear wp_clear_srv;

    //for set waypoint
    ros::ServiceClient wp_srv_client;
    mavros_msgs::WaypointPush wp_push_srv;

    //for set home location
    ros::ServiceClient set_home_client;
    mavros_msgs::CommandHome set_home_srv;
    
    //subscribe home_pos_sub
    ros::Subscriber home_pos_sub;//현재 subscribe를 계속 하고 있음, 최적화 하려면 키고 끄는 방식으로 바꿔야함
    void homePosCb(const mavros_msgs::HomePositionConstPtr& msg);

    ros::Subscriber wp_sub;
    mavros_msgs::WaypointList cur_wp;
    void wpCb(const mavros_msgs::WaypointListConstPtr& msg);

    //target gps
    _Float64 target_lat;
    _Float64 target_lon;

    //target gps
    _Float64 home_lat;
    _Float64 home_lon;

    //add waypoint
    void addWP(_Float64 lat, _Float64 lon, _Float64 alt);

    //add landing point
    void addLand(_Float64 lat, _Float64 lon, _Float64 alt);

    void calSubWP();

public:
    WpGenerator(ros::NodeHandle &nh);

    inline void setTarget(_Float64 lat, _Float64 lon){
        target_lat = lat;
        target_lon = lon;
        calSubWP();
    }

    bool detectTarget();

    //push waypoint
    bool pushWP();

    //set home position to current gps
    bool current2Home();

    //clear waypoint
    bool cleanWP();

};

}



#endif