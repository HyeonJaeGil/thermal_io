#pragma once 
#include <opencv2/opencv.hpp>

class CameraInfo
{
private:
    std::string camera_name;
    int image_width; 
    int image_height;
    cv::Mat camera_matrix;
    cv::Mat dist_coeff;
    
public:
    CameraInfo();
    std::string CameraName() const;
    void CameraName(std::string camera_name_);
    int Width() const;
    void Width(int image_width_);
    int Height() const;
    void Height(int image_height_);
    cv::Mat CameraMatrix() const;
    void CameraMatrix(cv::Mat camera_matrix_);
    void CameraMatrix(std::vector<double> camera_info_vec);
    cv::Mat DistCoeff() const;
    void DistCoeff(cv::Mat dist_coeff_);
    void DistCoeff(std::vector<double> dist_coeff_vec);
};