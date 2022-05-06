#include "elsed_detector.h"


void ElsedDetector::DetectLineFeature(cv::Mat& img_in)
{
    orig_img = img_in.clone();
    segs = elsed.detect(img_in);
    std::cout << "ELSED detected: " << segs.size() << " (large) segments" << std::endl;

    drawSegments(img_in, segs, CV_RGB(0, 255, 0), 2);
    // cv::imshow("ELSED long", img_in);
}