/* Information of UAV
 * include sensor like camera, lidar
 * size of frame
 * turn angle, etc
 */
#ifndef __UAV_H
#define __UAV_H
#include "pathmaker/Camera.h"
#include <iostream>

class UAV{
private:
    //horison, vertical, depth
    const double frameSize[3];
    
    //Addition distance for safe
    double safeDist;

    //we can use vector for  scalability
    Camera camera;
public:
    UAV(const double frameSize[3], const double safeDist, const Camera camera)
        : frameSize{frameSize[0], frameSize[1], frameSize[2]}
        , safeDist(safeDist)
        , camera(camera)
    {

    }

    void printInfo(){
        std::cout<<"(horison, vertical, depth) = ("
            <<frameSize[0]<<", "<<frameSize[1]<<", "<<frameSize[2]<<")"<<std::endl;
    }

   
    Camera& getCamera(){
        return this->camera;
    }

    
};


#endif