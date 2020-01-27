#ifndef KEYBOARDINPUT_H
#define KEYBOARDINPUT_H

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include "ros/ros.h"
#include "test/key.h"

class keyboardInput{
public:
    enum INPUT {NO, W, A, S, D, UP, DOWN, LEFT, RIGHT};
    keyboardInput();
    keyboardInput(bool isPrint);
    ~keyboardInput();
    void printInput();
    bool isPrint;
private:
    void init();
    void run();
    void end();

    struct termios org_term;
    struct termios new_term;
    enum INPUT lastInput;

    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Rate* rate;
    test::key msg;
};

#endif