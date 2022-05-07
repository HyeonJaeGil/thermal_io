#include "elsed_detector.h"


void ElsedDetector::DetectLineFeature(cv::Mat img_in)
{
    orig_img = img_in.clone();
    segs = elsed.detect(img_in);
    std::cout << "ELSED detected: " << segs.size() << " (large) segments" << std::endl;

    // cv::imshow("ELSED long", img_in);
}

void ElsedDetector::ShowDetectedImage(char* window_title)
{
    detected_img = orig_img.clone();
    drawSegments(detected_img, segs, CV_RGB(0, 255, 0), 2);
    cv::imshow(window_title, detected_img);

}
