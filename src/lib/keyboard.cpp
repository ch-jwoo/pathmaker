#include "pathmaker/keyboard.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "ros/ros.h"
#include "pathmaker/key.h"

keyboardInput::keyboardInput(){
    this->isPrint = false;
    this->init();
    this->run();
}

keyboardInput::keyboardInput(bool Print)
:isPrint(Print){
    this->init();
    this->run();
}

keyboardInput::~keyboardInput(){
    this->end();
}

void keyboardInput::init(){
    struct termios org_term; // for backup original terminal atrribute
    tcgetattr(STDIN_FILENO, &org_term);

    struct termios new_term = org_term; // new terminal atrribute
    new_term.c_lflag &= ~(ECHO | ICANON); // remove echo, canonical(wait until enter)
    new_term.c_cc[VMIN] = 0; // minimum character length set to zero
    new_term.c_cc[VTIME] = 0; // time out set to zero

    tcsetattr(STDIN_FILENO, TCSANOW, &new_term); // the attribute apply to terminal

    pub = n.advertise<pathmaker::key>("keyboardInput/key", 1000);
    rate = new ros::Rate(20.0);
}

void keyboardInput::run(){
    char input = 0;
    char dummy = 0;
    while(ros::ok()){
        //read(STDIN_FILENO, &dummy, 100);
        if(read(STDIN_FILENO, &input, 1) != 1) lastInput = NO;
        else{
            if(input == 27){
                if(read(STDIN_FILENO, &input, 1))
                    if(input==91)
                        if(read(STDIN_FILENO, &input, 1))
                            switch(input){
                                case 65:
                                    lastInput = UP;
                                    break;
                                case 68:
                                    lastInput = LEFT;
                                    break;
                                case 66:
                                    lastInput = DOWN;
                                    break;
                                case 67:
                                    lastInput = RIGHT;
                                    break;
                            }
                while(read(STDIN_FILENO, &dummy, 1) == 1);
            }
            else{
                switch(input){
                    case 'W':
                    case 'w':
                        lastInput = W;
                        break;
                    case 'A':
                    case 'a':
                        lastInput = A;
                        break;
                    case 'S':
                    case 's':
                        lastInput = S;
                        break;
                    case 'D':
                    case 'd':
                        lastInput = D;
                        break;
                }
            }
            if(isPrint == true)
                printInput();
        }
        
        msg.key = lastInput;
        pub.publish(msg);
        ros::spinOnce();
        rate->sleep();
    }
}

void keyboardInput::end(){
    tcsetattr(STDIN_FILENO, TCSANOW, &org_term); // terminal atrribute set to original attribute
}

void keyboardInput::printInput(){
    switch(lastInput){
        case W:
            fputs("W\n", stdout);
            break;
        case A:
            fputs("A\n", stdout);
            break;
        case S:
            fputs("S\n", stdout);
            break;
        case D:
            fputs("D\n", stdout);
            break;
        case UP:
            fputs("UP\n", stdout);
            break;
        case DOWN:
            fputs("DOWN\n", stdout);
            break;
        case LEFT:
            fputs("LEFT\n", stdout);
            break;
        case RIGHT:
            fputs("RIGHT\n", stdout);
            break;
    }
}