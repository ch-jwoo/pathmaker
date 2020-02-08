#ifndef __WPGENERATOR_H
#define __WPGENERATOR_H

#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/HomePosition.h>

namespace pm{

class WpGenerator{
private:
    //for wp clear
    ros::ServiceClient wp_clear_client;
    mavros_msgs::WaypointClear wp_clear_srv;

    //for set waypoint
    ros::ServiceClient wp_srv_client;
    mavros_msgs::WaypointPush wp_push_srv;

    //for set home location
    ros::ServiceClient set_home_client;
    mavros_msgs::CommandHome set_home_srv;
    
    //subscribe home position
    ros::Subscriber home_pos_sub;
    mavros_msgs::HomePosition home_pos;

    //target gps
    _Float64 target_lat;
    _Float64 target_lon;

    //target gps
    _Float64 home_lat;
    _Float64 home_lon;



    void calSubWP();
    void addWP(_Float64 lat, _Float64 lon, _Float64 alt);
    void addLand(_Float64 lat, _Float64 lon, _Float64 alt);

public:
    // WpGenerator();
    WpGenerator(_Float64 lat, _Float64 lon);

    inline void setTarget(_Float64 lat, _Float64 lon){
        target_lat = lat;
        target_lon = lon;
    }

    void homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg);
    
    //push waypoint
    void pushWP();

    //set home position to current gps
    void current2Home();

    //clear waypoint
    void cleanWP();

    //init : set home position, clear waypoint, push
    void init();
};




}



#endif