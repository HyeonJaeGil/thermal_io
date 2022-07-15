#pragma once
#include "thermal_io/thermal_io.h"


class Thermal8bitTransformer : public ThermalIO
{
public:
    Thermal8bitTransformer();
    Thermal8bitTransformer(std::string config_file);
    cv::Mat thermal14bit28bit(const cv::Mat img1);
    void img_cb(sensor_msgs::ImageConstPtr img_in) override;

};