#ifndef __CAMERA_H
#define __CAMERA_H
#include <ros/ros.h>

namespace pm{

//true -> false : delay some Duration
//false -> true : immediately
class delayFlag
{
private:
    bool flag;
    bool lastFlag;
    ros::Time lastTime;
    ros::Duration dur;
public:
    flagChecker(bool flag = false, double dur = 5.0)
        : dur(dur)
        , flag(flag)
        , lastFlag(flag)
    {

    }
    bool check(bool originalFlag)
    {
        if(originalFlag == false)
        {
            flag = false;
        }
        else
        {
            if(originalFlag != lastFlag)
            {
                lastTime = ros::Time::now();
            }

            if(ros::Time::now()- lastTime > dur)
            {
                flag = true;
            }
        }
        return flag;
    }

    inline bool getFlag()
    {
        return flag;
    }
}
}


#endif