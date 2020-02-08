#ifndef __MASTER_H
#define __MASTER_H

#include "localPathPlanning.h"
#include "wpGenerator.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

namespace pm{

/**
 * @brief
 * 
 **/
class Master{
private:
    ros::Rate rate;

    ros::Subscriber state_sub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    const mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;


    ros::Time last_request;

    LocalPathPlanning lp;
    WpGenerator wpG;

    const bool is_gzb;

    void terminalInput();
    
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    void update(const ros::TimerEvent &e);

    void setModOffb();

    void setModMission();


public:
    Master(const bool is_gzb = false);
    ~Master() {};

    void spin();

    inline void setPose(const geometry_msgs::PoseStamped &pose){
        this->pose = pose;
    }
};

}

#endif