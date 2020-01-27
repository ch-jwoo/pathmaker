#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pathmaker/key.h"
#include "pathmaker/keyboard.h"
#include <sstream>

int main(int argc, char** argv){
    //node name
    ros::init(argc, argv, "keyboard_node");

    
    keyboardInput input = keyboardInput(true);

    return 0;

}