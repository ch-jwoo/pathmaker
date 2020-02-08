#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <vector>
#include <cstring>

/* 0 : drone
 * 1 : camera typhoon_h480::cgo3_camera_link
 */

// void subModel(const gazebo_msgs::ModelStates::ConstPtr& msg){
//     model = *msg;
// }ddddd

void printTranform(const tf::Transform& temp){
    printf("position : (%f, %f, %f)\n", temp.getOrigin().getX(), temp.getOrigin().getY(), temp.getOrigin().getZ());
    printf("rotation : (%f, %f, %f, %f)\n", temp.getRotation().getX(), temp.getRotation().getY(), temp.getRotation().getZ(), temp.getRotation().getW());
}

tf::Transform pose2tf(const geometry_msgs::Pose &pose){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    transform.setRotation(q);

    return transform;
}

void subLink(const gazebo_msgs::LinkStates::ConstPtr& msg){
    static tf::TransformBroadcaster br;
    int i = 0;
    /* 4. base_link
     * 5. mount_link
     * 6. gimbal_vertical
     * 7. gimbal_horizontal
     * 8. cgo3_camera_link
     */
    int drone = 0;
    int camera = 0;
    while(true){
        if(!msg->name[i].compare("typhoon_h480::base_link")){//4
            drone = i;
        }
        if(!msg->name[i].compare("typhoon_h480::cgo3_camera_link")){//8
            camera = i;
            break;
        }
        i++;
    }
    // printf("drone, camera number : %d, %d\n", drone, camera);
    // tf::Transform transformBase;
    // const geometry_msgs::Pose &tempBase = msg->pose[base];
    // transformBase.setOrigin(tf::Vector3(tempBase.position.x, tempBase.position.y, tempBase.position.z));
    // tf::Quaternion qBase(tempBase.orientation.x, tempBase.orientation.y, tempBase.orientation.z, tempBase.orientation.w);
    // transformBase.setRotation(qBase);

    // tf::Transform groundTest;
    // groundTest.setOrigin(tf::Vector3(0, 0, 0));
    // tf::Quaternion qTest;
    // qTest.setRPY(0,0,0);
    // groundTest.setRotation(qTest);
    
    static tf::Transform groundTf = pose2tf(msg->pose[drone]);//starting point

    tf::Transform droneTf = pose2tf(msg->pose[drone]);
    
    tf::Transform offsetTf; // camera offset
    offsetTf.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(-3.141592/2,0,-3.141592/2);
    offsetTf.setRotation(q);

    tf::Transform cameraTf = pose2tf(msg->pose[camera]);
    cameraTf *= droneTf.inverse() * offsetTf;

    q.setRPY(0,0,3.141592); // drone offset
    offsetTf.setRotation(q);
    droneTf *= groundTf.inverse();

    br.sendTransform(tf::StampedTransform(groundTf, ros::Time::now(), "map", "ground"));
    br.sendTransform(tf::StampedTransform(droneTf, ros::Time::now(), "ground", "drone"));
    br.sendTransform(tf::StampedTransform(cameraTf, ros::Time::now(), "drone", "camera_link"));


    // tf::Transform transformBase;
    // const geometry_msgs::Pose &tempBase = msg->pose[base];
    // transformBase.setOrigin(tf::Vector3(tempBase.position.x, tempBase.position.y, tempBase.position.z));
    // tf::Quaternion qBase(tempBase.orientation.x, tempBase.orientation.y, tempBase.orientation.z, tempBase.orientation.w);
    // transformBase.setRotation(qBase);

    // br.sendTransform(tf::StampedTransform(transformBase, ros::Time::now(), "map_ned", "camera_link"));

    // tf::Transform transformCamera;
    // const geometry_msgs::Pose &tempCamera = msg->pose[camera];
    // transformCamera.setOrigin(tf::Vector3(tempCamera.position.x, tempCamera.position.y, tempCamera.position.z));
    // tf::Quaternion qCamera(tempCamera.orientation.x, tempCamera.orientation.y, tempCamera.orientation.z, tempCamera.orientation.w);
    // transformCamera.setRotation(qCamera);

    // tf::Transform lense;
    // lense.setOrigin(tf::Vector3(-0.051, 0, -0.162));
    // tf::Quaternion qLense;
    // qLense.setRPY(0, 0, 3.14159);
    // lense.setRotation(qCamera);

    // transformCamera *= transformBase.inverse();
    // transformCamera *= lense;
    // br.sendTransform(tf::StampedTransform(transformCamera, ros::Time::now(), "map_ned", "camera_link"));
    
}
int main(int argc, char** argv){
    ros::init(argc, argv, "tf_node");
    ros::NodeHandle nh;
    ros::Subscriber subL = nh.subscribe<gazebo_msgs::LinkStates>
          ("gazebo/link_states", 10, subLink);


    ros::Rate rate(10.0);

    ros::spin();
    return 0;
};