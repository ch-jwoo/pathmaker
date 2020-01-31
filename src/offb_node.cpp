#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/Thrust.h"
#include "std_msgs/Float32.h"
#include "mavros/mavros.h"
#include <cmath>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
 mavros_msgs::State current_state;
//#include </home/mahesh/catkin_ws/src/beginner_tutorials/src/Qualisys.h>
 void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "offb_node");
   ros::NodeHandle n;
   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
   ros::Publisher pub_att = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",10);
   ros::Publisher pub_thr = n.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 10);
   //ros::Publisher param_set=n.advertise<mavros_msgs::ParamSet>("/mavros/param/push")
   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
   ros::Rate loop_rate(100);
   
   
   geometry_msgs::PoseStamped cmd_att;
   mavros_msgs::Thrust cmd_thr;
   //mavros_msgs::ParamSet cmd_param;
   std_msgs::Header head;
   mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
   mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
   
   ros::Time last_request = ros::Time::now();

  
   int count = 1;
   double v[3]={0.0, 0.0, 1.0};
   double lambda = 0.45;
   head.stamp = ros::Time::now();
   head.seq = count;
   head.frame_id = 1;
   
   double theta=1.0; 
   double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
   //double theta=0.0;
     cmd_thr.header = head;
     cmd_thr.thrust = lambda;
     cmd_att.header.stamp = ros::Time::now();
     cmd_att.header.seq=count;
     cmd_att.header.frame_id = 1;
     cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
     cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
     cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;
 
     cmd_att.pose.orientation.x = sin(theta/2.0)*v[0]/v_norm;
     cmd_att.pose.orientation.y = sin(theta/2.0)*v[1]/v_norm;
     cmd_att.pose.orientation.z = sin(theta/2.0)*v[2]/v_norm;
     cmd_att.pose.orientation.w = cos(theta/2.0);
           
    while(ros::ok()){
        /*if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
            }
         else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                    ROS_INFO("trying to arm....");
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                }*/
        
       pub_att.publish(cmd_att);
       pub_thr.publish(cmd_thr);      
       
       ros::spinOnce();
        count++;
	   theta=0.3*(count/300.0);
       ROS_INFO("deg: %f",theta);
       loop_rate.sleep();
        }
        
       
       

 return 0;
        
}

   
    
       
  
