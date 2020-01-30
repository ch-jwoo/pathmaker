#include "pathmaker/ImgProc.h"
#include <ros/ros.h>
#include "pathmaker/UAV.h"

int main(int argc, char** argv){
    Camera camera(87.0, 58.0, 1000.0, 20.0);
    const double frameSize[3] = {650, 240, 240}; // horizon : 650mm, vertical : 240mm
    
    UAV uav(frameSize, 100.0, camera);
    uav.printInfo();
    uav.getCamera().printInfo();
    
    ros::init(argc, argv, "imgproc_node");
    ImgProc imgproc(30.0, uav, true, true, true);//30hz
    return 0;

}