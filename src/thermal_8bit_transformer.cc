#include "thermal_io/thermal_8bit_transformer.h"


Thermal8bitTransformer::Thermal8bitTransformer(std::string config_file)
    : ThermalIO(config_file)
{
    img_sub = nh.subscribe(in_topic, 1, &Thermal8bitTransformer::img_cb, this);
    img_pub = nh.advertise<sensor_msgs::Image>(out_topic, 10);
}

void Thermal8bitTransformer::img_cb(sensor_msgs::ImageConstPtr img_in)
{
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::MONO16);
    int row = cv_ptr->image.rows;
    int col = cv_ptr->image.cols;
    cv::Mat new_image(row, col, CV_8UC1);

    bool undistort_image = false;

    if(undistort_image){
        cv::Mat img_undistort(cv_ptr->image.rows, cv_ptr->image.cols, CV_16UC1);
        assert(!cv_ptr->image.empty());
        cv::undistort(cv_ptr->image, img_undistort, cam_info.CameraMatrix(), cam_info.DistCoeff());
        for(int i = 0; i < row; ++i)
            for(int j = 0; j < col; ++j){
                auto pixel = img_undistort.at<ushort>(i,j);
                // std::cout << i <<", " << j << ": " << pixel << std::endl;
                new_image.at<u_char>(i,j) = uchar(pixel / 64);
            }
    }
    else {
        for(int i = 0; i < row; ++i)
            for(int j = 0; j < col; ++j){
                auto pixel = cv_ptr->image.at<ushort>(i,j);
                // std::cout << i <<", " << j << ": " << pixel << std::endl;
                new_image.at<u_char>(i,j) = uchar(pixel / 64);
            }
    }

    cv_bridge::CvImage out_msg;
    out_msg.header   = img_in->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
    out_msg.image    = new_image; // Your cv::Mat

    img_pub.publish(out_msg.toImageMsg());
}