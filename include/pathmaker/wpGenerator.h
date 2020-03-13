#ifndef __WPGENERATOR_H
#define __WPGENERATOR_H

#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointList.h>
// #include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>

namespace pm{
//make sub waypoint from destination GPS
class WpGenerator{
private:
    ros::NodeHandle &nh;
    sensor_msgs::NavSatFix &cur_global_pose;

    mavros_msgs::WaypointPush wp_push_data;

    //target gps
    double target_lat;
    double target_lon;

    int max_mission;

    //add waypoint
    void addWP(double lat, double lon, double alt);

    //add landing point
    void addLand(double lat, double lon, double alt);

    void calSubWP();

public:
    WpGenerator(ros::NodeHandle &nh, sensor_msgs::NavSatFix &cur_global_pose);

    inline void setTarget(double lat, double lon){
        target_lat = lat;
        target_lon = lon;
        calSubWP();
    }

    bool detectTarget();

    //push waypoint
    bool pushWP();

    //clear waypoint
    bool cleanWP();

    bool isMissionComplete();
};

}



#endif