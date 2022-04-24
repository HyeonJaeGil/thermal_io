#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "yaml-cpp/yaml.h"

// #include <boost/bind.hpp>


class ThermalIO
{
public:
    ThermalIO();
    ThermalIO(std::string config_file);
    void img_cb(sensor_msgs::ImageConstPtr img_in);

private:
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
    int image_width; 
    int image_height;
    cv::Mat camera_matrix;
    cv::Mat dist_coeff;

    bool ReadConfigFile(YAML::Node& config_file);
};

ThermalIO::ThermalIO()
    :it(nh)
{
    // img_sub = nh.subscribe<sensor_msgs::Image>(in_topic, 100, boost::bind(img_cb, _1));
    config_file = YAML::LoadFile
            ("/home/hj/line_ws/src/thermal_io/config/thermal_14bit_left.yaml");
    ReadConfigFile(config_file);
    in_topic = "/thermal_14bit_left/image_raw";
    out_topic = "/thermal_8bit_left/image_raw";
    img_sub = nh.subscribe(in_topic, 1, &ThermalIO::img_cb, this);
    img_pub = nh.advertise<sensor_msgs::Image>(out_topic, 10);
}

void ThermalIO::img_cb(sensor_msgs::ImageConstPtr img_in)
{
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::MONO16);
    cv::Mat img_undistort(image_width, image_height, CV_16UC1);
    cv::undistort(cv_ptr->image, img_undistort, camera_matrix, dist_coeff);
    int row = cv_ptr->image.rows;
    int col = cv_ptr->image.cols;
    cv::Mat new_image(row, col, CV_8UC1);
    for(int i = 0; i < row; ++i)
        for(int j = 0; j < col; ++j){
            auto pixel = img_undistort.at<ushort>(i,j);
            // std::cout << i <<", " << j << ": " << pixel << std::endl;
            new_image.at<u_char>(i,j) = uchar(pixel / 64);
        }
    cv::Mat equalized_image(row, col, CV_8UC1);
    cv::equalizeHist(new_image, equalized_image);
    cv::imshow("view", equalized_image);   
     
    cv::Mat equalized_image_3ch(row, col, CV_8UC3);
    // cv::equalizeHist(new_image, equalized_image_3ch);
    // cv::imshow("view2", equalized_image_3ch);

    cv::cvtColor(equalized_image, equalized_image_3ch, CV_GRAY2BGR, 3);

    cv_bridge::CvImage out_msg;
    out_msg.header   = img_in->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
    out_msg.image    = equalized_image; // Your cv::Mat

    img_pub.publish(out_msg.toImageMsg());
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", equalized_image_3ch).toImageMsg();
    // img_pub.publish(msg);

    return;
}

bool ThermalIO::ReadConfigFile(YAML::Node& config_file)
{
    const std::string camera_name = config_file["camera_name"].as<std::string>();
    image_width = config_file["image_width"].as<int>();
    image_height = config_file["image_height"].as<int>();    

    camera_matrix = cv::Mat
            (config_file["camera_matrix"]["data"].as<std::vector<double>>());
    dist_coeff = cv::Mat
            (config_file["distortion_coefficients"]["data"].as<std::vector<double>>()).reshape(1,5);

    camera_matrix = (cv::Mat_<double>(3, 3) << 529.4328871, 0.0, 311.119, 
                                                0.0, 529.5314275019, 266.1281757546, 
                                                0.0, 0.0, 1.0);
    dist_coeff = (cv::Mat_<double>(1, 5) << -3.580882335e-01, 9.94318453e-02, 0, 0, 0);

    std::cout << "camera matrix: \n" << camera_matrix<< std::endl;
    std::cout << "distortion_coefficients: \n" << dist_coeff<< std::endl;
    return true;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "thermal_io_node");
    cv::namedWindow("view");
    cv::namedWindow("view2");
    cv::startWindowThread();
    ThermalIO thermal_io_node;
    ROS_INFO("Starting thermal_io_node ...");
    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("view2");
    return 0;

}