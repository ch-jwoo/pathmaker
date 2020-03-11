#include "pathmaker/master.h"
#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Float64.h>

namespace pm{

Master::Master()
    : nh()
    , lp(nh, curPose)
    , wpG(nh, cur_global_pose)
{
    while(!ros::topic::waitForMessage<mavros_msgs::State>
                ("mavros/state", ros::Duration(5.0))){
        ROS_ERROR("There is no \"mavros/state\" message");
    }
    while(!ros::topic::waitForMessage<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose", ros::Duration(5.0))){
        ROS_ERROR("There is no \"mavros/local_position/pose\" message");
    }
    // while(!ros::topic::waitForMessage<sensor_msgs::NavSatFix>
    //             ("mavros/global_position/global", ros::Duration(5.0))){
    //     ROS_ERROR("There is no \"mavros/global_position/global\" message");
    // }

    stateSub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 1, &Master::stateCb, this);
    poseSub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 1, &Master::poseCb, this);
    global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>
        ("mavros/global_position/global", 1, &Master::globalPoseCb, this);

    posePub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 1);
}

void Master::setMode(int eMode){
    static ros::ServiceClient setModeClient 
            = nh.serviceClient<mavros_msgs::SetMode>
                    ("mavros/set_mode", true);
    static mavros_msgs::SetMode targetMode;

    targetMode.request.custom_mode = MODE[eMode];
    setModeClient.call(targetMode);
    std::cout<<"Mode change request : " << MODE[eMode] << std::endl;
}

void Master::setArm(bool arming){
    static ros::ServiceClient armingClient 
            = nh.serviceClient<mavros_msgs::CommandBool>
                            ("mavros/cmd/arming", true);
    static mavros_msgs::CommandBool armCmd;

    armCmd.request.value = arming;
    if(armingClient.call(armCmd) && armCmd.response.success){
        ROS_INFO("Vehicle %s", arming ? "armed" : "disarmed");
    }
    // armingClient.call(armCmd);
    // ROS_INFO("Vehicle ariming request : %s", arming? "True": "False");
}

void Master::modeCb(const ros::TimerEvent &e){
    static std::string lastMode = getCurMode(); // for returning to origin mode
    static DelayFlag obstacleFlag; // offboard terminate after 5s of obstacle disappearing

    posePub.publish(lp.getTargetPose());
    obstacleFlag.check(lp.obstacleDetected());
    //ROS_INFO("%f", getAlt());
    if(getCurMode() == MODE[MISSION]){
        if(obstacleFlag.getFlag() && getAlt()>1.5){ // obstacle detected
            setMode(OFFBOARD);
            lastMode = getCurMode();
            ROS_INFO("OFFBOARD");
        }
    }
    else if(getCurMode() == MODE[OFFBOARD]){//avoidance
        if(!obstacleFlag.getFlag()){
            setMode(findModeNum(lastMode));
            ROS_INFO("ESCAPE");
        }
    }
    else if(getCurMode() == MODE[MAN]){
            ROS_INFO("current mode : position");
        if(obstacleFlag.getFlag() && getAlt()>2.0){ // obstacle detected
            setMode(OFFBOARD);
            lastMode = getCurMode();
            ROS_INFO("OFFBOARD");
        }
    }
    else if(getCurMode() == MODE[FORCEDMAN]){
        /*empty*/
    }
}


void Master::spin(){
    while(ros::ok() && !curState.connected){
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Mavros connected");
    // waitTarget();
    // initialArming();//offboard && arm
    
    ros::AsyncSpinner spinner(4 /* threads */);

    auto modeCheckTimer = nh.createTimer(
            ros::Duration(0.1), &Master::modeCb, this);
    spinner.start();
    
    ros::waitForShutdown();
	ROS_INFO("Stopping path planning...");
	spinner.stop();
}

void Master::waitTarget(){
    while(!wpG.cleanWP()){
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }
    while(!wpG.detectTarget()){
        ros::spinOnce();
        ros::Duration(10.0).sleep();
    }
    while(!wpG.pushWP()){
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }
}

void Master::initialArming(){
    ros::Time lastRequest = ros::Time::now();
    int step = 0;
    ros::Rate rate(20.0);

    while(getCurMode() != MODE[MISSION] || !armCheck()){
        posePub.publish(curPose);
        if(step == 0){
            if(getCurMode() != MODE[OFFBOARD])
                setMode(OFFBOARD);
            else if(!armCheck()){
                setArm(true);
            }
            else{
                step = 1;
            }
        }
        else if(step == 1){
            if(!armCheck()){
                step = 0;
            }
            else{
                setMode(MISSION);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    // pubTimer.stop();
}

int Master::findModeNum(std::string mode){
    int num = -1;
    for(int i=0; i<NUMOFMODE; i++){
        if(mode == MODE[i]){
            return i;
        }
    }
    ROS_ERROR("mode mismatch in array");
    exit(-1);
}

void Master::stateCb(const mavros_msgs::State::ConstPtr& msg){
    curState = *msg;
}
void Master::poseCb(const geometry_msgs::PoseStampedConstPtr& msg){
    curPose = *msg;
}
void Master::globalPoseCb(const sensor_msgs::NavSatFixConstPtr& msg){
    cur_global_pose = *msg;
}


}

// 참고
// auto diag_timer = nh.createTimer(//spinning func
// 		ros::Duration(0.5),
// 		[&](const ros::TimerEvent &) {

// 		});
// diag_timer.start(); 


// ros::Timer diag_timer = nh.createTimer(//spinning func
// 		ros::Duration(0.1), &Master::update, this);
