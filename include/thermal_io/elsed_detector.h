#include "line_detector_interface.h"
#include "thirdparty/ELSED/src/ELSED.h"

class ElsedDetector : public LineDetectorInterface
{
private:
    cv::Mat orig_img;
    cv::Mat detected_img;
    upm::ELSED elsed;
    upm::Segments segs;
    upm::ELSEDParams params;

    void drawSegments(cv::Mat img,
                    upm::Segments segs,
                    const cv::Scalar &color,
                    int thickness = 1,
                    int lineType = cv::LINE_AA,
                    int shift = 0)
    {
        for (const upm::Segment &seg: segs)
            cv::line(img, cv::Point2f(seg[0], seg[1]), cv::Point2f(seg[2], seg[3]), 
                    color, thickness, lineType, shift);
    }

public:
    ElsedDetector() = default;
    ElsedDetector(cv::Mat img) : orig_img(img){}
    ElsedDetector(upm::ELSEDParams param) : elsed(param){}
    virtual void DetectLineFeature(cv::Mat img_in) override;
    virtual void ShowDetectedImage(char* window_title) override;


};