#ifndef __SWITCH_H
#define __SWITCH_H
extern "C"
{
#include "jetsonGPIO.h"
}
class Switch
{
    jetsonGPIO pushButton;

public:
    Switch(int gpio = gpio216) : pushButton(gpio)
    {
        gpioExport(pushButton);
        gpioSetDirection(pushButton,inputPin) ;
    }

    ~Switch()
    {
        gpioExport(pushButton);
    }

    unsigned int getSwitch(){
        unsigned int value;
        gpioGetValue(pushButton, &value);
	    printf("switch : %d\n", value);
        return value;
    }
    enum {ATTACH = 0, DETACH = 1};
};

# endif
