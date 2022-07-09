#include "thermal_io/line_detector_interface.h"
#include <opencv2/line_descriptor.hpp>

class LsdDetector : public LineDetectorInterface
{
private:
    cv::Mat orig_img;
    cv::Mat detected_img;
    cv::Ptr<cv::line_descriptor::LSDDetector> bd;
    std::vector<cv::line_descriptor::KeyLine> lines;
    cv::Mat mask;


public:
    LsdDetector();
    virtual void DetectLineFeature(cv::Mat img_in) override;
    virtual void ShowDetectedImage(char* window_title) override;


};