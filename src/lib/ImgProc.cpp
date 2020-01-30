#include "pathmaker/ImgProc.h"

//read setting.txt, set window
ImgProc::ImgProc(double rate, const UAV &uav, bool cvtWindow, bool fromGzb, bool prWindow)
    :rate(rate)
    ,ict(nh, img, cvtWindow, fromGzb)
    ,uav(uav)
    ,pr(img, uav, prWindow)
{
    //ict.ImageConverter(nh, img, true, true);//open Window, for gzb
    //cmdToOffb.publish(msg);
    this->init();
    this->run();
}

ImgProc::~ImgProc(){
}

void ImgProc::init(){
}
void ImgProc::run(){
    while(ros::ok()){
        if(!img.empty()){
            pr.updatePR(img);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void ImgProc::end(){
    
}