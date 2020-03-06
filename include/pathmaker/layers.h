#ifndef __LAYERS_H
#define __LAYERS_H

#include <opencv2/core.hpp>

namespace pm{

class Layers{
private:
    cv::Mat bin;
    cv::Mat cost;
    cv::Mat dist;

    float azimuth;
    float elevation;

    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    //param
    float Kd;
    float Kh;
    float Ke;

    const int ORIGIN_WIDTH;
    const int ORIGIN_HEIGHT;
    const int LAYERS_WIDTH;
    const int LAYERS_HEIGHT;
    const double FOV_H;
    const double FOV_V;
    const double THRESHOLD;
    const double MAXDISTANCE;

public:
    Layers();
    ~Layers() = default;

    void update(cv::Mat &img);

    inline float getAzimuth(){
        return azimuth;
    }
    inline float getElevation(){
        return elevation;
    }
};

}
#endif
