#include <opencv2/opencv.hpp>
#include "camera_info.h"

CameraInfo::CameraInfo()
    : camera_matrix(cv::Mat(3, 3, CV_64FC1)), dist_coeff(cv::Mat(1, 5, CV_64FC1))
{}

cv::Mat CameraInfo::CameraMatrix() const {return camera_matrix;}
void CameraInfo::CameraMatrix(cv::Mat camera_matrix_) {camera_matrix = camera_matrix_;}    
void CameraInfo::CameraMatrix(std::vector<double> camera_info_vec) 
{
    memcpy(camera_matrix.data, camera_info_vec.data(), camera_info_vec.size()*sizeof(double));
    std::cout << "camera matrix: \n" << camera_matrix<< std::endl;
}    

cv::Mat CameraInfo::DistCoeff() const {return dist_coeff;}
void CameraInfo::DistCoeff(cv::Mat dist_coeff_) {dist_coeff = dist_coeff_;}
void CameraInfo::DistCoeff(std::vector<double> dist_coeff_vec)
{
    memcpy(dist_coeff.data, dist_coeff_vec.data(), dist_coeff_vec.size()*sizeof(double));
    std::cout << "distortion_coefficients: \n" << dist_coeff<< std::endl;
}

std::string CameraInfo::CameraName() const {return camera_name;}
void CameraInfo::CameraName(std::string camera_name_) {camera_name = camera_name_;}

int CameraInfo::Width() const {return image_width;}
void CameraInfo::Width(int image_width_) {image_width =image_width_;}

int CameraInfo::Height() const {return image_height;}
void CameraInfo::Height(int image_height_) {image_height =image_height_;}

