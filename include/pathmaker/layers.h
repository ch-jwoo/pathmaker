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
    float Kd = 4.0f;
    float Kh = 1.0f;
    float Ke = 1.0f;

    const int ORIGIN_WIDTH = 848;
    const int ORIGIN_HEIGHT = 480;
    const int LAYERS_WIDTH = 9;
    const int LAYERS_HEIGHT = 15;
    const double FOV_H = 87.0;
    const double FOV_V = 58.0;
    const double THRESHOLD = 5.0;
    const double MAXDISTANCE = 10.0;

public:
    Layers();
    ~Layers() = default;

    void update(const cv::Mat &img);

    inline float getAzimuth(){
        return azimuth;
    }
    inline float getElevation(){
        return elevation;
    }
};

}
#endif