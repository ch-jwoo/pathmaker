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
        //std::cout<<"original img = "<<img.rows<<" * "<<img.cols<<std::endl;
        if(!img.empty()){
            pr.updateROI(img);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void ImgProc::end(){
    
}