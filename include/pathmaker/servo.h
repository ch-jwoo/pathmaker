#ifndef __SERVO_H
#define __SERVO_H

#include "JHPWMPCA9685.h"
#define PWM 60
class Servo
{
private:  
    PCA9685* pca9685;
    int servoMin;
    int servoMax;

    bool status;

public:
    enum {CLOSED = 0, OPENED = 1};

    Servo(int min = 120, int max = 720)
        : servoMin(min)
        , servoMax(max)
    {
        pca9685 = new PCA9685();
        int err = pca9685->openPCA9685();
        if(err < 0){
            printf("Error: %d", pca9685->error);
            exit(1);
        }
        else{
            printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
            pca9685->setAllPWM(0,0);
            pca9685->reset();
            pca9685->setPWMFrequency(PWM);
        }
        this->open();
    }

    void close()
    {
        pca9685->setPWM(0,0,servoMin);//open
        status = CLOSED;
    }

    void open()
    {
        pca9685->setPWM(0,0,map(95,0,180,servoMin, servoMax)) ;//close
        status = OPENED;
    }

    bool getStatus() { return status; }
};

# endif