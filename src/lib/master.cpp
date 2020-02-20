#include "pathmaker/master.h"
#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Float64.h>

namespace pm{

Master::Master()
    : lp(nh, curPose)
    , wpG(nh)
{
    stateSub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, &Master::stateCb, this);
    posePub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 1);

    //persistent connection
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming", true);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode", true);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
}

void Master::setMode(int eMode){
    switch (eMode)
    {
    case OFFBOARD:
        targetMode.request.custom_mode = "OFFBOARD";
        break;
    case MISSION:
        targetMode.request.custom_mode = "AUTO.MISSION";
        break;
    default:
        ROS_ERROR("setMode Error");
        break;
    }
}

// void Master::update(const ros::TimerEvent &e){
//     lp.update();
//     // ros::spinOnce();
// }

void Master::spin(){
    ros::AsyncSpinner spinner(4 /* threads */);

    last_request = ros::Time::now();
    ros::Timer diag_timer = nh.createTimer(//spinning func
			ros::Duration(0.1), &Master::update, this);
    

    // auto diag_timer = nh.createTimer(//spinning func
	// 		ros::Duration(0.5),
	// 		[&](const ros::TimerEvent &) {

	// 		});
    
	diag_timer.start();
	spinner.start();
	// ros::waitForShutdown();

    wpG.setTarget(47.4042079, 8.5757766);
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

    setArm(true);
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
    while(ros::ok()){
        if(lp.getCheck() && rel_alt.data>2.0){
            // local_pos_pub.publish(lp.getPose());
            set_mode.request.custom_mode = "OFFBOARD";
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(set_mode) &&
                    set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }
        else if(current_state.mode != "AUTO.MISSION" || !current_state.armed)
        {
            set_mode.request.custom_mode = "AUTO.MISSION";
            if( current_state.mode != "AUTO.MISSION" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(set_mode) &&
                    set_mode.response.mode_sent){
                    ROS_INFO("AUTO.MISSION enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

	ROS_INFO("Stopping path planning...");
	spinner.stop();
}

void Master::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void Master::altcb(const std_msgs::Float64::ConstPtr& msg)
{
    rel_alt = *msg;
}
}