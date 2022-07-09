#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "camera_info.h"
#include "thermal_io/elsed_detector.h"
#include "thermal_io/lsd_detector.h"
// #include <boost/bind.hpp>


class ThermalIO
{
public:
    ThermalIO();
    ThermalIO(std::string config_file);
    void img_cb(sensor_msgs::ImageConstPtr img_in);
    cv::Mat thermal14bit28bit(const cv::Mat img1);

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
    CameraInfo cam_info;
    // LineDetectorInterface* line_detector;
    ElsedDetector elsed;
    LsdDetector lsd;


    bool ReadConfigFile(YAML::Node& config_file);
};

ThermalIO::ThermalIO()
    :it(nh)
{
    // img_sub = nh.subscribe<sensor_msgs::Image>(in_topic, 100, boost::bind(img_cb, _1));
    config_file = YAML::LoadFile
            ("/home/hj/Workspace/thermal_ws/src/thermal_io/config/thermal_14bit_left.yaml");
    ReadConfigFile(config_file);
    // in_topic = "/thermal_14bit_left/image_raw";
    // out_topic = "/thermal_8bit_left/image_raw";
    img_sub = nh.subscribe(in_topic, 1, &ThermalIO::img_cb, this);
    img_pub = nh.advertise<sensor_msgs::Image>(out_topic, 10);
    // line_detector = &elsed;
}

void ThermalIO::img_cb(sensor_msgs::ImageConstPtr img_in)
{
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::MONO16);
    cv::Mat img_undistort(cv_ptr->image.rows, cv_ptr->image.cols, CV_16UC1);
    assert(!cv_ptr->image.empty());
    cv::undistort(cv_ptr->image, img_undistort, cam_info.CameraMatrix(), cam_info.DistCoeff());
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
    // cv::imshow("view", equalized_image);   
     
    cv::Mat equalized_image_3ch(row, col, CV_8UC3);
    cv::cvtColor(equalized_image, equalized_image_3ch, CV_GRAY2BGR, 3);

    lsd.DetectLineFeature(equalized_image_3ch);
    lsd.ShowDetectedImage("view2");

    elsed.DetectLineFeature(equalized_image_3ch);
    elsed.ShowDetectedImage("view3");
    
    cv_bridge::CvImage out_msg;
    out_msg.header   = img_in->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
    out_msg.image    = equalized_image; // Your cv::Mat

    img_pub.publish(out_msg.toImageMsg());
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", equalized_image_3ch).toImageMsg();
    // img_pub.publish(msg);

    return;
}


cv::Mat ThermalIO::thermal14bit28bit(const cv::Mat img1)
{
    const float PB = 1428, PF = 1.0, PO = 118.126, PR = 377312.0; 
    float min_T0, max_T0, min_T1, max_T1, min_T2, max_T2;
	cv::Mat temp = img1.clone();
    cv::Size imageSize= img1.size(); 
    cv::Mat map1, map2;
	initUndistortRectifyMap(cam_info.CameraMatrix(), cam_info.DistCoeff(), cv::Mat(), 
                            cam_info.CameraMatrix(), imageSize, CV_32FC1, map1, map2);
	remap(img1, temp, map1, map2, cv::INTER_LINEAR);
	// cv::undistort(img1, img1, camera_info.cvK, camera_info.cvD);
	cv::Mat undist1, hist_img;
	cv::Mat temimg = cv::Mat::zeros(imageSize, CV_8UC(100));
	auto lowVal0=PR/(exp(PB/(273.15+min_T0))-PF)+PO;
	auto highVal0=PR/(exp(PB/(273.15+max_T0))-PF)+PO;
	auto countRange0 = highVal0 - lowVal0;
	auto lowVal1=PR/(exp(PB/(273.15+min_T1))-PF)+PO;
	auto highVal1=PR/(exp(PB/(273.15+max_T1))-PF)+PO;
	auto countRange1 = highVal1 - lowVal1; 
	auto lowVal2=PR/(exp(PB/(273.15+min_T2))-PF)+PO;
	auto highVal2=PR/(exp(PB/(273.15+max_T2))-PF)+PO;
	auto countRange2 = highVal2 - lowVal2;  
	const double PI = 3.1415926;
	// char buffer[256];
	// char now[256];
	// sprintf(buffer, "%06d", n);
	

	// std::string save_imgfilename1 = "/home/jyk/aa/thermal_image/"+ folder + "/left/" + buffer + ".yml";
	// cv::FileStorage fs1(save_imgfilename1, cv::FileStorage::WRITE);

	#pragma omp parallel
	for(int channel =0; channel < 100; ++channel)
	{
		auto lowVal = PR/(exp(PB/(273.15+10))-PF)+PO;
		auto highVal = PR/(exp(PB/(273.15+14+0.3*channel))-PF)+PO;
		auto countRange = highVal - lowVal;  
		for (int count1 = 0; count1 < 640; ++count1)
		{
			for (int count2 = 0; count2 < 512; ++count2)
			{
				int tmpData1 = temp.at<uint16_t>(count2,count1);
				int n0=floor(((tmpData1- lowVal)*256.0 / countRange));
				if(n0>=0)
				{	
					if(floor(128 * (sin( (((tmpData1- lowVal)/countRange)-1) * PI / 2) )+128)<256)
					{
						// temimg.at<Vec100b>(count2,count1)[channel]=floor(128(sin((((tmpData1- lowVal) / countRange)-1)PI/2))+128);

					}
					else
					{
						// temimg.at<Vec100b>(count2,count1)[channel]=255;
					}
				}
			}
		}
		// sprintf(now, "%d", channel);
		// fs1 << "matrix" << temimg;
		// fs1.write("matrix",thermal8bitimg1);
		
	}
	// fs1.release();
	// if(do_flip==1)
	// {
	// 	cv::flip(temimg, temimg, -1);
	// }
	
	// equalizeHist(undist1, hist_img);
	undist1 = temimg;

	return undist1;
}

bool ThermalIO::ReadConfigFile(YAML::Node& config_file)
{
    cam_info.CameraName(config_file["camera_name"].as<std::string>());
    cam_info.Width(config_file["image_width"].as<int>());
    cam_info.Height(config_file["image_height"].as<int>());
    cam_info.CameraMatrix(config_file["camera_matrix"]["data"].as<std::vector<double>>());
    cam_info.DistCoeff(config_file["distortion_coefficients"]["data"].as<std::vector<double>>());

    in_topic = config_file["in_topic"].as<std::string>();
    out_topic = config_file["out_topic"].as<std::string>();

    // std::cout << "camera matrix is :" << cam_info.CameraMatrix()<< std::endl;
    // std::cout << "distortion coefficient is :" << cam_info.DistCoeff()<< std::endl;
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