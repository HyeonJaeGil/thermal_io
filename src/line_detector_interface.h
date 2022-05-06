#pragma once

#include <opencv2/opencv.hpp>

class LineDetectorInterface
{
public:
    virtual void DetectLineFeature(cv::Mat& img_in) = 0; 

private:
    
    
};