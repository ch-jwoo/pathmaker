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

#include "delayFlag.h"

namespace pm{

/**
 * @brief
 * 
 **/
class Master{
private:
    // static const std::string OFFBOARD;
    // static const std::string MISSION;

    enum eMode{OFFBOARD, MISSION, MAN, FORCEDMAN};//unordered map 이용해서 구현하면 더 좋을 듯
    std::string MODE[4] = {"OFFBOARD", "AUTO.MISSION", "POSCTL", "ALTCTL"};

    ros::NodeHandle nh;

    //subscribe, publish, service
    ros::Subscriber stateSub;
    mavros_msgs::State currentState;
    void stateCb(const mavros_msgs::State::ConstPtr& msg);

    ros::Subscriber poseSub;
    geometry_msgs::PoseStamped curPose;//NED
    void poseCb(const geometry_msgs::PoseStampedConstPtr& msg);

    ros::Publisher posePub;

    ros::ServiceClient armingClient;
    mavros_msgs::CommandBool armCmd;

    ros::ServiceClient setModeClient;
    mavros_msgs::SetMode targetMode;

    //for checking obstacle
    DelayFlag obstacleFlag;

    //for obstacle avoidance
    LocalPathPlanning lp;

    //for mission
    WpGenerator wpG;
    
    // inline bool obstacleCheck(){
    //     /* logic */
    //     return true;
    // }
    inline double getAlt(){
        return curPose.pose.position.z;
    }
    inline bool armCheck(){
        return currentState.armed;
    }
    inline std::string getCurMode(){
        return currentState.mode;
    }
    void setTarget(double lat, double lon);

    void setMode(int eMode);
    void setMode(std::string mode);

    void setArm(bool arming);

    void modeCb(const ros::TimerEvent &e);

    void targetPosePubCb(const ros::TimerEvent &e);


    /**
     * set Mode offboard, set arm
     * 
     **/
    void initialArming();

public:
    Master();
    ~Master() {};

    void spin();
};


// const std::string Master::OFFBOARD = "OFFBOARD";
// const std::string Master::MISSION = "AUTO.MISSION";

}

#endif