#ifndef __MASTER_H
#define __MASTER_H

#include "localPathPlanning.h"
#include "wpGenerator.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstring>
#include <std_msgs/Float64.h>


namespace pm{

/**
 * @brief
 * 
 **/
class Master{
private:
    const std::string OFFBOARD = "OFFBOARD";
    const std::string MISSION = "AUTO.MISSION";
    
    ros::NodeHandle nh;
    ros::Rate rate;
    ros::Time last_request;

    ros::Subscriber state_sub;
    mavros_msgs::State current_state;

    ros::Subscriber alt_sub;
    std_msgs::Float64 rel_alt;

    ros::ServiceClient arming_client;
    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient set_mode_client;
    mavros_msgs::SetMode set_mode;

    // ros::Publisher local_pos_pub;

    LocalPathPlanning lp;
    WpGenerator wpG;
    
    const bool is_gzb;

    void terminalInput();
    
    void stateCb(const mavros_msgs::State::ConstPtr& msg);

    void altcb(const std_msgs::Float64::ConstPtr& msg);

    void update(const ros::TimerEvent &e);
    
    


    inline std::string getTargetMode() const
    {
        return set_mode.request.custom_mode;
    }

    inline std::string getCurrentMode() const
    {
        return current_state.mode;
    }

    inline void setTargetMode(const std::string &str){
        set_mode.request.custom_mode = str;
    }

    inline void setArm(const bool arming)
    {
        arm_cmd.request.value = arming;
    }

public:
    Master(const bool is_gzb = false);
    ~Master() {};

    void spin();
};

}

#endif