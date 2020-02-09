#ifndef __LAYERS_H
#define __LAYERS_H

#include <opencv2/core.hpp>

namespace pm{

class Layers{
private:
    cv::Mat bin;
    cv::Mat add_cost;
    cv::Mat dist;
    const int width_block;
    const int height_block;
    const float fov_h;
    const float fov_v;
    const float threshold;

public:
    //in real world
    Layers();

    ~Layers() = default;

    void update(const cv::Mat &depthMap);
};

}
#endif