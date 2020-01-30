/* Information of Camera
 * camera spec, position
 */

#ifndef __CAMERA_H
#define __CAMERA_H
#include <iostream>

class Camera{
private:
    //degree
    double degH;
    double degV;

    //mm
    double minDist;
    double maxDist;

public:
    Camera(double degH, double degV, double minDist, double maxDist)
        : degH(degH)
        , degV(degV)
        , minDist(minDist)
        , maxDist(maxDist)
    {

    }
    Camera(const Camera &camera){
        *this = camera;
    }

    void printInfo(){
        std::cout<<"(degH, degV) = ("<<degH<<", "<<degV<<")"<<std::endl;
    }

    double getDegH(){ return degH; }
    double getDegV(){ return degV; }
    double getMinDist(){ return minDist; }
    double getMaxDist(){ return maxDist; }
};

#endif