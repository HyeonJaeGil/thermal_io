#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "thermal_io/camera_info.h"
// #include <boost/bind.hpp>

class ThermalIO
{
public:
    ThermalIO();
    ThermalIO(std::string config_file_);
    virtual void img_cb(sensor_msgs::ImageConstPtr img_in) = 0;

protected:
    ros::NodeHandle nh;
    std::string in_topic;
    std::string out_topic;
    ros::Subscriber img_sub;
    ros::Publisher img_pub;
    image_transport::ImageTransport it;
    // image_transport::Subscriber image_sub;
    // image_transport::Publisher image_pub;
    cv_bridge::CvImagePtr cv_ptr;
    YAML::Node config_file;
    CameraInfo cam_info;
    int max_value, min_value;
    int crop_height;

protected:
    bool ReadConfigFile(YAML::Node& config_file);
};