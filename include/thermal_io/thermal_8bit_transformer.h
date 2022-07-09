#pragma once
#include "thermal_io/thermal_io.h"


class Thermal8bitTransformer : public ThermalIO
{
public:
    Thermal8bitTransformer();
    Thermal8bitTransformer(std::string config_file);
    void img_cb(sensor_msgs::ImageConstPtr img_in) override;

};