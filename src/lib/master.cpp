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
    , wpG(nh)
{
    stateSub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 1, &Master::stateCb, this);
    posePub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 1);

    //persistent connection
    armingClient = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming", true);
    setModeClient = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode", true);

}

void Master::setMode(int eMode){
    targetMode.request.custom_mode = MODE[eMode];
    setModeClient.call(targetMode);
    // if( setModeClient.call(targetMode) && getCurMode() == MODE[eMode] ){
    //     std::cout<<"Mode change : " << MODE[eMode] << std::endl;
    // }
}
void Master::setMode(std::string mode){
    targetMode.request.custom_mode = mode;
    setModeClient.call(targetMode);
    // if( setModeClient.call(targetMode) && getCurMode() == mode ){
    //     std::cout<<"Mode change : " << mode << std::endl;
    // }
}


void Master::setArm(bool arming){
    armCmd.request.value = arming;
    if(armingClient.call(armCmd) && armCmd.response.success){
        ROS_INFO("Vehicle %s", arming ? "armed" : "disarmed");
    }
}
// void Master::update(const ros::TimerEvent &e){
//     lp.update();
//     // ros::spinOnce();
// }


void Master::setTarget(double lat, double lon){
    wpG.setTarget(lat, lon);
    // while(!wpG.current2Home()){
    //     ros::spinOnce();
    //     ros::Duration(5.0).sleep();
    // }
    while(!wpG.cleanWP()){
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }
    while(!wpG.pushWP()){
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }
}

void Master::modeCb(const ros::TimerEvent &e){
    static auto pubTimer = nh.createTimer(
                            ros::Duration(0.1),
                            [&](const ros::TimerEvent &) {
                                posePub.publish(lp.getTargetPose());
                            },
                            false, 
                            false);
    static ros::Time modeRequest = ros::Time::now();
    static ros::Time armRequest = ros::Time::now();
    static std::string lastMode = getCurMode();
    static ros::Rate rate(20.0);

    obstacleFlag.check(lp.obstacleDetected());
    // if(obstacleFlag.getFlag() && getAlt()>2.0){ // OFFBOARD
    //     pubTimer.start();
    //     if( getCurMode() != MODE[OFFBOARD] && // current mode != offboard
    //         ros::Time::now() - modeRequest > ros::Duration(5.0)){
    //         setMode(OFFBOARD);
    //         modeRequest = ros::Time::now();
    //     }
    // }
    // else{//MISSION
    //     pubTimer.stop();
    //     if( getCurMode() != MODE[MISSION] && // current mode != mission
    //         ros::Time::now() - modeRequest > ros::Duration(5.0)){
    //         setMode(MISSION);
    //         modeRequest = ros::Time::now();
    //     }
    // }
    
    
    if(getCurMode() == MODE[MISSION]){
        std::cout<<"MISSION"<<std::endl;
        if(obstacleFlag.getFlag() && getAlt()>2.0){ // obstacle detected
            pubTimer.start();
            setMode(OFFBOARD);
            lastMode = getCurMode();
        }
    }
    else{
        pubTimer.stop();

        if(getCurMode() == MODE[OFFBOARD]){//avoidance
            std::cout<<"OFFBOARD"<<std::endl;
            if(!obstacleFlag.getFlag()){
                setMode(lastMode);
            }
        }
        else if(getCurMode() == MODE[MAN]){
            std::cout<<"MANUAL"<<std::endl;
            if(obstacleFlag.getFlag() && getAlt()>2.0){ // obstacle detected
                pubTimer.start();
                setMode(OFFBOARD);
                lastMode = getCurMode();
            }
        }
        else if(getCurMode() == MODE[FORCEDMAN]){
            std::cout<<"FORCED MANUAL"<<std::endl;
            /*empty*/
        }
    }
    if(!armCheck() && ros::Time::now() - armRequest > ros::Duration(5.0)){
        setArm(true);
    }
    ros::spinOnce();
    rate.sleep();
}


void Master::spin(){
    while(ros::ok() && !currentState.connected){
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Mavros connected");

    ros::AsyncSpinner spinner(4 /* threads */);

    // ros::Timer diag_timer = nh.createTimer(//spinning func
	// 		ros::Duration(0.1), &Master::update, this);
    

    // auto diag_timer = nh.createTimer(//spinning func
	// 		ros::Duration(0.5),
	// 		[&](const ros::TimerEvent &) {

	// 		});
	// diag_timer.start(); 
	
    spinner.start();
    
    setTarget(47.4042079, 8.5757766);//waypoint set

    initialArming();//offboard && arm

    auto modeCheckTimer = nh.createTimer(
            ros::Duration(0.1), &Master::modeCb, this);
    
    ros::waitForShutdown();
    // wpG.setTarget(47.4042079, 8.5757766);
    // while(!wpG.current2Home()){
    //     ros::spinOnce();
    //     ros::Duration(5.0).sleep();
    // }
    // while(!wpG.cleanWP()){
    //     ros::spinOnce();
    //     ros::Duration(5.0).sleep();
    // }
    // while(!wpG.pushWP()){
    //     ros::spinOnce();
    //     ros::Duration(5.0).sleep();
    // }

    // setArm(true);
    // set_mode.request.custom_mode = "AUTO.MISSION";
    // while(current_state.mode != "AUTO.MISSION" || !current_state.armed)
    // {
    //     if( current_state.mode != "AUTO.MISSION"){
    //         if( set_mode_client.call(set_mode) &&
    //             set_mode.response.mode_sent){
    //             ROS_INFO("AUTO.MISSION enabled");
    //         }
    //     } else {
    //         if( !current_state.armed){
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success){
    //                 ROS_INFO("Vehicle armed");
    //             }
    //         }
    //     }
    //     ros::spinOnce();
    //     ros::Duration(5.0).sleep();
    // }

    // ros::spin();

	ROS_INFO("Stopping path planning...");
	spinner.stop();
}

void Master::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
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
}

}