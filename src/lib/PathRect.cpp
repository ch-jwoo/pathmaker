#include "pathmaker/PathRect.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
 ::LocalPathPlanning(const cv::Mat &img, const UAV &uav, bool openWindow)
    : uav(uav)
    , openWindow(openWindow)
{
    if(openWindow){
        cv::namedWindow(WINDOW_NAME);
    }
}

void LocalPathPlanning::updatePR(const cv::Mat &notEmptyImg){
    
    img = notEmptyImg.clone();   
    //std::cout<<img.rows<<" * "<<img.cols<<std::endl;
    //pathRect = cv::Rect_<int>(img.cols/4.0, img.rows/4.0, img.cols/2, img.rows/4.0);
    //std::cout<<pathRect.tl().x<<", "<<pathRect.tl().y<<", "<<pathRect.br().x<<", "<<pathRect.br().y<<std::endl;
    
    imgROI = cv::Mat(img, pathRect);
    
    cv::minMaxLoc(imgROI, &minDepth, &maxDepth, &minLoc, &maxLoc);

    calAvg();
    std::cout<<"avgDepth = "<<avgDepth<<std::endl;

    if(openWindow){
        cv::Mat showImg = img.clone();
        cv::rectangle(showImg, pathRect.tl(), pathRect.br(), cv::Scalar(0, 0, 0), 3);
        cv::imshow(WINDOW_NAME, showImg);
        cv::waitKey(5);
    }
}

void PathRect::calRect(){
    
}

void PathRect::calAvg(){
    size_t sum = 0;
    for(int i = 0; i<imgROI.rows; i++){
        for(int j = 0; j<imgROI.cols; j++){
            sum += imgROI.at<uint16_t>(i, j);
        }
    }

    avgDepth = (double)sum/ (imgROI.rows * imgROI.cols);
}