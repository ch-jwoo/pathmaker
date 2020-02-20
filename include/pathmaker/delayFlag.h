#ifndef __DELAYFLAG_H
#define __DELAYFLAG_H
#include <ros/ros.h>

namespace pm{

//true -> false : delay some Duration
//false -> true : immediately
class DelayFlag
{
private:
    bool flag;
    bool lastFlag;
    ros::Time lastTime;
    ros::Duration dur;
public:
    DelayFlag(bool flag = false, double dur = 5.0)
        : dur(dur)
        , flag(flag)
        , lastFlag(flag)
    {

    }
    bool check(bool inputFlag)
    {
        if(flag == false){
            flag = inputFlag;
        }
        else{
            if(lastFlag != inputFlag){
                lastTime = ros::Time::now();
            }
            else{
                if(ros::Time::now() - lastTime > dur){
                    flag = true;
                }
            }
        }
        lastFlag = inputFlag;
        return flag;
    }

    inline bool getFlag()
    {
        return flag;
    }
};


}


#endif