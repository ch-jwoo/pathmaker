#ifndef __MASTER_H
#define __MASTER_H

#include "localPathPlanning.h"
#include "wpGenerator.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>

#include "delayFlag.h"
#include "servo.h"
#include "switch.h"

#define NUMOFMODE 4

namespace pm{

/**
 * @brief
 * 
 **/
class Master{
private:
    enum eMode{OFFBOARD, MISSION, MAN, FORCEDMAN};//unordered map 이용해서 구현하면 더 좋을 듯
    std::string MODE[NUMOFMODE] = {"OFFBOARD", "AUTO.MISSION", "POSCTL", "ALTCTL"};
    int findModeNum(std::string mode);
    
    ros::NodeHandle nh;

    //subscribe, publish, service
    ros::Subscriber stateSub;
    mavros_msgs::State curState;
    void stateCb(const mavros_msgs::State::ConstPtr& msg);

    ros::Subscriber poseSub;
    geometry_msgs::PoseStamped curPose;//NED
    void poseCb(const geometry_msgs::PoseStampedConstPtr& msg);

    ros::Subscriber global_pose_sub;
    sensor_msgs::NavSatFix cur_global_pose;//global
    void globalPoseCb(const sensor_msgs::NavSatFixConstPtr& msg);

    ros::Publisher posePub;

    //for obstacle avoidance
    LocalPathPlanning lp;

    //for mission
    WpGenerator wpG;

    Servo servo;
    
    Switch swit;

    void waitTarget();
    
    //set Mode offboard, set arm
    void initialArming();

    void modeCb(const ros::TimerEvent &e);
    void servoCb(const ros::TimerEvent &e);
    void wpCb(const ros::TimerEvent &e);
    void setArm(bool arming);
    void setMode(int eMode);

    void targetPosePubCb(const ros::TimerEvent &e);
public:
    Master();
    ~Master() {};

    void spin();

    
    inline double getAlt(){
        return curPose.pose.position.z;
    }
    inline bool armCheck(){
        return curState.armed;
    }
    inline std::string getCurMode(){
        return curState.mode;
    }
};


}

#endif