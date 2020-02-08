/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "pathmaker/key.h"
#include "pathmaker/keyboard.h"
#include <math.h>


mavros_msgs::State current_state;
pathmaker::key current_key;
geometry_msgs::Twist vel;

void calSpeed(double x, double y, double z, double rz){
    vel.linear.x = x;
    vel.linear.y = y;
    vel.linear.z = z;
    vel.angular.z = rz;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void state_key(const pathmaker::key::ConstPtr& msg){
    current_key = *msg;
    switch(msg->key){
        case keyboardInput::W: // W, S : alttitude vel.linear.z
            calSpeed(0,0,5,0);
            break;
        case keyboardInput::S:
            calSpeed(0,0,-5,0);
            break;
        case keyboardInput::A: // A, D : YAW vel.angular.z
            calSpeed(0,0,0,5);
            break;
        case keyboardInput::D:
            calSpeed(0,0,0,-5);
            break;
        case keyboardInput::UP: // UP, DOWN : vel.linear.x
            calSpeed(5,0,0,0);
            break;
        case keyboardInput::DOWN:
            calSpeed(-5,0,0,0);
            break;
        case keyboardInput::LEFT: // LEFT, RIGHT : vel.linear.y
            calSpeed(0,5,0,0);
            break;
        case keyboardInput::RIGHT:
            calSpeed(0,-5,0,0);
            break;
        default:
            calSpeed(0,0,0,0);
            break;
    }

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
           ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber keyInput = nh.subscribe<pathmaker::key>
            ("keyboardInput/key", 10, state_key);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
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

        //local_pos_pub.publish(pose);
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}