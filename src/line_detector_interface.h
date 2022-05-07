#pragma once

#include <opencv2/opencv.hpp>

class LineDetectorInterface
{
public:
    virtual void DetectLineFeature(cv::Mat img_in) = 0; 
    virtual void ShowDetectedImage(char* window_title) = 0;
private:
    
    
};