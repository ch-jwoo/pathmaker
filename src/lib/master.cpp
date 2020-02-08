#include "pathmaker/master.h"
#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

namespace pm{

Master::Master(const bool is_gzb)
    : is_gzb(is_gzb)
    , rate(20.0)
    , lp(is_gzb)
{
    ros::NodeHandle nh;

    stateSub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
            
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //waypoint mode
    wpG.init(); // push the WP

    arm_cmd.request.value = true;
    setMission();
}

void Master::update(const ros::TimerEvent &e){
    lp.update();
    if(lp.getCheck()){
        if(current_state.mode != "OFFBOARD"){
            setOffb();
        }
        lp.publish();
    }
    else{
        if(current_state.mode != "AUTO.MISSION"){
            setMission();
        }
    }
}

void Master::spin(){
    ros::AsyncSpinner spinner(4 /* threads */);


    auto diag_timer = nh.createTimer(//spinning func
			ros::Duration(0.5), this->update);
    

    // auto diag_timer = nh.createTimer(//spinning func
	// 		ros::Duration(0.5),
	// 		[&](const ros::TimerEvent &) {

	// 		});
    
	diag_timer.start();
	spinner.start();
	ros::waitForShutdown();

	ROS_INFO("Stopping path planning...");
	spinner.stop();
}


void Master::setModOffb(){
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Rate rate(20.0);
    while(current_state.mode != "OFFBOARD" || !current_state.armed){
        last_request = ros::Time::now();
        local_pos_pub.publish(pose);

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard mode");
            }
        }
        else if( !current_state.armed){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Master::setModMission(){
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    while(current_state.mode != "AUTO.MISSION" || !current_state.armed){
        local_pos_pub.publish(pose);
        if( current_state.mode != "AUTO.MISSION"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Mission flight mode");
            }
        }
        else if( !current_state.armed){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
        }
        ros::spinOnce();
        ros::Duration().sleep();
    }
}

}