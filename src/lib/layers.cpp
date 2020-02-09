#include "pathmaker/layers.h"
#include <opencv2/core.hpp>

namespace pm{

Layers::Layers()
    : height_block(9)
    , width_block(15)
    , fov_h(87)
    , fov_v(58)
    , threshold(5.0)
    , bin(height_block,width_block,CV_32F)
    , dist(height_block,width_block,CV_32F)
    , add_cost(height_block,width_block,CV_32F)
{

}

void Layers::update(const cv::Mat &depthMap){
    // if(isGzb){
    //     for(int k=0;k<height_block;k++){
    //         for(int l=0;l<width_block;l++){
    //             for(int i=0;i<height_cut;i++){
    //                 for(int j=0;j<width_cut;j++)
    //                 {
    //                   depth_cal.at<float>(k,l)+=image.at<float>(i+1+k*height_cut,j+4+l*width_cut);
    //                   depth_cost.at<float>(k,l)+=18-image.at<float>(i+1+k*height_cut,j+4+l*width_cut);
    //                 }
    //             }
    //             if(depth_cal.at<float>(k,l)/(width_cut*height_cut) - margin_dist<0)
    //             {
    //                 binary_cal.at<uint8_t>(k,l)=1;
    //             }
    //             else
    //                 binary_cal.at<uint8_t>(k,l)=0;//평균으로 mat을 만들면 아주 작은물체가 가까이있으면 인식을 못할수도 있음
    //         }
    //     }///for
    //     //depth_cal=~depth_cal;
    //    depth = depth_cal/(width_cut*height_cut);
    //    depth_cost=depth_cost/(width_cut*height_cut);
    //    binary = binary_cal;
    // }//callback
}

}