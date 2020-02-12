#include "pathmaker/layers.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <cmath>
namespace pm{

Layers::Layers()
    : LAYERS_HEIGHT(9)
    , LAYERS_WIDTH(15)
    , FOV_H(87)
    , FOV_V(58)
    , THRESHOLD(5.0)
    , ORIGIN_WIDTH(848)
    , ORIGIN_HEIGHT(480)
    , MAXDISTANCE(10.0)
{
    bin = cv::Mat::zeros(LAYERS_HEIGHT,LAYERS_WIDTH,CV_8UC1);
    dist = cv::Mat::zeros(LAYERS_HEIGHT,LAYERS_WIDTH,CV_32F);
    cost = cv::Mat::zeros(LAYERS_HEIGHT,LAYERS_WIDTH,CV_32F);
    rect = cv::Rect2i(0,0,ORIGIN_WIDTH/LAYERS_WIDTH, ORIGIN_HEIGHT/LAYERS_HEIGHT);
}

void Layers::update(const cv::Mat &img){

    // for(int i=0; rect.tl().y <= img.cols;)
    // {
    //     for(int j=0; rect.br().x <= img.rows;){
    //         dist.at<double> = cv::mean(cv::Mat(img, rect));
    //         rect(rect.tl().x + rect.size().width, rect.tl().y + rect.size().height);
    //     }
    // }
    int width_cut = ORIGIN_WIDTH/LAYERS_WIDTH;
    int height_cut = ORIGIN_HEIGHT/LAYERS_HEIGHT; 
    for(int k=0;k<LAYERS_HEIGHT;k++)
    {
        for(int l=0;l<LAYERS_WIDTH;l++)
        {
            dist.at<float>(k,l) = 0; // initialize
            for(int i=0; i<height_cut; i++){
                for(int j=0; j<width_cut; j++){
                    dist.at<float>(k,l)+=img.at<float>(i+1+k*height_cut,j+4+l*width_cut);
                }
            }
            //average
            dist.at<float>(k,l) /= (width_cut*height_cut);
            if(dist.at<float>(k,l) < THRESHOLD){
                bin.at<uint8_t>(k,l)=1;
            }
            else{
                bin.at<uint8_t>(k,l)=0;
            }
            dist.at<float>(k,l) = 10 - dist.at<float>(k,l);
        }
    }

    for(int k=0; k<9; k++)
    {
        for(int l=0; l<15; l++)
        {
            if(bin.at<uint8_t>(k,l)==1)
            {
                if(bin.at<uint8_t>(k-1,l+1)==0)//1 k-1 l+1
                {
                    if(k-1>=0&&l+1<=14)
                    {
                        bin.at<uint8_t>(k-1,l+1)=2;
                    }
                }
                
                if(bin.at<uint8_t>(k-1,l-1)==0)//2 k-1 l-1
                {
                    if(k-1>=0&&l-1>=0)
                    {
                        bin.at<uint8_t>(k-1,l-1)=2;
                    }
                }

                if(bin.at<uint8_t>(k-1,l)==0)//3  k-1 l
                {
                    if(k-1>=0)
                    {
                        bin.at<uint8_t>(k-1,l)=2;
                    }
                }
                if(bin.at<uint8_t>(k,l+1)==0)   // k l+1
                {
                    if(l+1<=14)
                    {
                        bin.at<uint8_t>(k,l+1)=2;
                    }
                }
                if(bin.at<uint8_t>(k,l-1)==0)   // k  l-1
                {
                    if(l-1>=0)
                    {

                        bin.at<uint8_t>(k,l-1)=2;
                    }
                }
                if(bin.at<uint8_t>(k+1,l)==0)//1  k+1 l
                {
                    if(k+1<=9)
                    {
                        bin.at<uint8_t>(k+1,l)=2;
                    }
                }
                if(bin.at<uint8_t>(k+1,l-1)==0)//2 k+1 l-1
                {
                    if(k+1<=9&&l-1>=0)
                    {
                        bin.at<uint8_t>(k+1,l-1)=2;
                    }
                }

                if(bin.at<uint8_t>(k+1,l+1)==0)//3 k+1 l+1
                {
                    if(k+1<=9&&l+1<=14)
                    {
                        bin.at<uint8_t>(k+1,l+1)=2;
                    }
                }
                
            }
        }
    }

    for(int i=0;i<9;i++)
    {
        for(int j=0;j<15;j++)
        {
            if(bin.at<uint8_t>(i,j))
            {
                bin.at<uint8_t>(i,j)=255;
            }
        }
    }

    cv::Mat blurred_depth_cost;
    cv::GaussianBlur(dist,blurred_depth_cost,cv::Size(9,9),1.0);                      ///////////////@#$%@$#%@$#%        블러 추가부분
    // std::cout <<blurred_depth_cost <<std::endl;

    for(int i=0; i<LAYERS_HEIGHT; i++){   ///i 0~7 8칸
        for(int j=0; j<LAYERS_WIDTH; j++){   ///j 0~13 14칸
            cost.at<float>(i,j) 
                    = Kd*blurred_depth_cost.at<float>(i,j)
                    + Kh*abs((FOV_H/2.0) - (FOV_H/LAYERS_WIDTH) * (j+1/2.0))
                    + Ke*abs((FOV_V/2.0) - (FOV_V/LAYERS_HEIGHT) * (((LAYERS_HEIGHT-1)-i)+1/2.0));
        }
    }

    // std::cout<<"bin: \n"<<bin<<std::endl<<std::endl;
    // std::cout<<"cost: \n"<<cost<<std::endl<<std::endl;

    cv::minMaxLoc(cost, &minVal, &maxVal, &minLoc, &maxLoc, ~bin);//얘가 문제
    printf("minLoc : %d, %d\n", minLoc.x, minLoc.y);
    this->azimuth= ((minLoc.x-7)*(FOV_H/15.0f))*(M_PI/180.0);//*(M_PI/180.0);  //degree
    this->elevation= ((4-minLoc.y)*(FOV_V/9.0f))*(M_PI/180.0);
    

    // std::cout<<cost<<std::endl;
    // std::cout<<minLoc<<std::endl;
}

}