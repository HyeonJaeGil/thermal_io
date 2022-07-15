#include "thermal_io/thermal_io.h"


ThermalIO::ThermalIO(std::string config_file_)
    :it(nh)
{
    config_file = YAML::LoadFile(config_file_);
    ReadConfigFile(config_file);
    // img_sub = nh.subscribe<sensor_msgs::Image>(in_topic, 100, boost::bind(img_cb, _1));

}

ThermalIO::ThermalIO()
    :ThermalIO("/home/hj/Workspace/thermal_io_ws/src/thermal_io/config/thermal_14bit_left.yaml")
{}

bool ThermalIO::ReadConfigFile(YAML::Node& config_file)
{
    cam_info.CameraName(config_file["camera_name"].as<std::string>());
    cam_info.Width(config_file["image_width"].as<int>());
    cam_info.Height(config_file["image_height"].as<int>());
    cam_info.CameraMatrix(config_file["camera_matrix"]["data"].as<std::vector<double>>());
    cam_info.DistCoeff(config_file["distortion_coefficients"]["data"].as<std::vector<double>>());

    in_topic = config_file["in_topic"].as<std::string>();
    out_topic = config_file["out_topic"].as<std::string>();

    max_value = config_file["max_value"].as<int>();
    min_value = config_file["min_value"].as<int>();
    crop_height = config_file["crop_height"].as<int>();
    visualize = config_file["visualize"].as<int>();

    // std::cout << "camera matrix is :" << cam_info.CameraMatrix()<< std::endl;
    // std::cout << "distortion coefficient is :" << cam_info.DistCoeff()<< std::endl;
    return true;
}