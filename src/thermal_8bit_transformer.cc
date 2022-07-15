#include "thermal_io/thermal_8bit_transformer.h"


Thermal8bitTransformer::Thermal8bitTransformer(std::string config_file)
    : ThermalIO(config_file)
{
    img_sub = nh.subscribe(in_topic, 1, &Thermal8bitTransformer::img_cb, this);
    img_pub = nh.advertise<sensor_msgs::Image>(out_topic, 10);
}

cv::Mat Thermal8bitTransformer::thermal14bit28bit(const cv::Mat img1)
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
	cv::Mat temimg = cv::Mat::zeros(imageSize, CV_8UC1);
	// auto lowVal0=PR/(exp(PB/(273.15+min_T0))-PF)+PO;
	// auto highVal0=PR/(exp(PB/(273.15+max_T0))-PF)+PO;
	// auto countRange0 = highVal0 - lowVal0;
	// auto lowVal1=PR/(exp(PB/(273.15+min_T1))-PF)+PO;
	// auto highVal1=PR/(exp(PB/(273.15+max_T1))-PF)+PO;
	// auto countRange1 = highVal1 - lowVal1; 
	// auto lowVal2=PR/(exp(PB/(273.15+min_T2))-PF)+PO;
	// auto highVal2=PR/(exp(PB/(273.15+max_T2))-PF)+PO;
	// auto countRange2 = highVal2 - lowVal2;  
	const double PI = 3.1415926;
	// char buffer[256];
	// char now[256];
	// sprintf(buffer, "%06d", n);
	

	// std::string save_imgfilename1 = "/home/jyk/aa/thermal_image/"+ folder + "/left/" + buffer + ".yml";
	// cv::FileStorage fs1(save_imgfilename1, cv::FileStorage::WRITE);

	// #pragma omp parallel
	for(int channel =0; channel < 1; ++channel)
	{
		auto lowVal = PR/(exp(PB/(273.15+15))-PF)+PO;
		auto highVal = PR/(exp(PB/(273.15+19+0.3*channel))-PF)+PO;
		auto countRange = highVal - lowVal;  
		for (int count1 = 0; count1 < 640; ++count1)
		{
			for (int count2 = 0; count2 < 512; ++count2)
			{
				int tmpData1 = temp.at<uint16_t>(count2,count1);
				int n0=floor(((tmpData1- lowVal)*256.0 / countRange));
				if(n0>=0)
				{	
					if(floor(128 * (sin( (((tmpData1- lowVal)/countRange)-1) * PI / 2) )+128) < 256)
					{
						temimg.at<int8_t>(count2,count1)=floor(128 * (sin((((tmpData1- lowVal) / countRange)-1)* PI/2))+128);

					}
					else
					{
						temimg.at<int8_t>(count2,count1)=255;
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

void Thermal8bitTransformer::img_cb(sensor_msgs::ImageConstPtr img_in)
{
    /*
        old_image -> old_image_undistorted -> old_image_cropped -> new_image_cropped
    */

    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::MONO16);
    int row = cv_ptr->image.rows;
    int col = cv_ptr->image.cols;
    cv::Mat old_image = (cv_ptr->image).clone();
    cv::Mat new_image(row, col, CV_8UC1);
    cv::Mat old_image_cropped(crop_height, col, CV_16UC1);
    cv::Mat new_image_cropped(crop_height, col, CV_8UC1);

    cv::Mat old_image_undistorted(old_image.size(), CV_16UC1);
    assert(!cv_ptr->image.empty());
    cv::undistort(old_image, old_image_undistorted, cam_info.CameraMatrix(), cam_info.DistCoeff());

    for(int i = 0; i < row; ++i)
        for(int j = 0; j < col; ++j)
        {
            if(i < crop_height) old_image_cropped.at<uint16_t>(i,j) = old_image_undistorted.at<uint16_t>(i,j);
        }


    bool sine_remapping = false;

    if(sine_remapping)
    {
        new_image = thermal14bit28bit(cv_ptr->image);
        cv::imshow("view", new_image);   
        cv::waitKey(1);
    }
    else {
        for(int i = 0; i < crop_height; ++i)
            for(int j = 0; j < col; ++j)
            {    
                if(old_image_cropped.at<uint16_t>(i,j) > max_value) old_image_cropped.at<uint16_t>(i,j) = max_value;
                else if(old_image_cropped.at<uint16_t>(i,j) < min_value) old_image_cropped.at<uint16_t>(i,j) = min_value;

                // auto pixel = cv_ptr->image.at<ushort>(i,j);
                // // std::cout << i <<", " << j << ": " << pixel << std::endl;
                // new_image.at<u_char>(i,j) = uchar(pixel / 64);
            }

        // normalize
        for(int i = 0; i < crop_height; i++)
        {
            for(int j = 0; j < col; j++)
            {
                new_image_cropped.at<uint8_t>(i,j) = 255*(old_image_cropped.at<uint16_t>(i,j) - min_value) / (max_value - min_value);
            }
        }
        cv::imshow(out_topic, new_image_cropped);
        cv::waitKey(1);
    }
    

    cv_bridge::CvImage out_msg;
    out_msg.header   = img_in->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
    out_msg.image    = new_image; // Your cv::Mat

    img_pub.publish(out_msg.toImageMsg());
}
