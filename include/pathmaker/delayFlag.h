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
    ros::Duration dur;
public:
    DelayFlag(bool flag = false, double dur = 5.0)
        : dur(dur)
        , flag(flag)
    {

    }
    bool check(bool inputFlag)
    {
        static ros::Time lastTime = ros::Time::now();
        static bool tab = false;
        if(flag == false){
            if(inputFlag == true)
                flag = true;
        }
        else{//flag==true
            if(inputFlag == false && tab == false){
                lastTime = ros::Time::now();
                tab = true;
            }
            if(inputFlag == true){
                tab = false;
            }
            if(inputFlag == false && ros::Time::now() - lastTime > dur)
                flag = false;
        }
        return flag;
    }

    inline bool getFlag()
    {
        return flag;
    }
};


}


#endif