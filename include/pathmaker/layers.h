#ifndef __LAYERS_H
#define __LAYERS_H

#include <opencv2/core.hpp>

namespace pm{

class Layers{
private:
    cv::Mat bin;
    cv::Mat cost;
    cv::Mat dist;

    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    float azimuth;
    float elevation;

    float Kd = 4.0f;
    float Kh = 1.0f;
    float Ke = 1.0f;

    const int ORIGIN_WIDTH;
    const int ORIGIN_HEIGHT;
    const int LAYERS_WIDTH;
    const int LAYERS_HEIGHT;
    const double FOV_H;
    const double FOV_V;
    const double THRESHOLD;
    const double MAXDISTANCE;

    cv::Rect2i rect;
public:
    //in real world
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