/**
 * @author George Terzakis
 * @create date 2021-06-18 17:10:02
 * @modify date 2021-06-18 17:10:02
 * @desc Zed stereo publisher
 */

#ifndef __ZED_STEREO_PUBLISHER__
#define __ZED_STEREO_PUBLISHER__

#include <iostream>
#include <fstream>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <termios.h>
#include <mutex>
#include <thread>
#include <memory>
#include <experimental/filesystem> // TODO: lose experimental with C++17

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/CameraInfo.h>           // For publishing camera calibration 
#include <image_transport/image_transport.h> // For image conversions to OpenCV Mat_<> / Mat
#include <cv_bridge/cv_bridge.h>             // For image conversions to OpenCV Mat_<> / Mat
#include <sensor_msgs/image_encodings.h>     // For image conversions to OpenCV Mat_<> / Mat


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace std {
namespace filesystem = experimental::filesystem;
}

namespace fs = std::filesystem;

namespace slam_image_publisher 
{

    struct CameraCalibrationEntry
    {
        double fx, fy, cx, cy;
        double k1, k2, k3;
        double p1, p2;
    };

    class ZedStereoPublisher {

    public:

        ZedStereoPublisher(
            const ros::NodeHandle& nh, 
            const ros::NodeHandle& nhp, 
            const std::string& left_info_topic_name = "/zed/left/camera_info",
		    const std::string& left_image_topic_name = "/zed/left/image_rect", 
		    const std::string& right_info_topic_name = "/zed/right/camera_info",
		    const std::string& right_image_topic_name = "/zed/right/image_rect");
    
        ~ZedStereoPublisher();

    private:

        std::string left_video_filename_;
        std::string right_video_filename_;
        std::string calibration_filename_;
        
        std::string left_info_topic_name_;
        std::string right_info_topic_name_;
        std::string left_image_topic_name_;
        std::string right_image_topic_name_;
        
        std::shared_ptr<ros::Publisher> pleft_info_pub_;
        std::shared_ptr<ros::Publisher> pright_info_pub_;
        std::shared_ptr<image_transport::Publisher> pleft_image_pub_;
        std::shared_ptr<image_transport::Publisher> pright_image_pub_;
        
        // Video capture members
        std::shared_ptr<cv::VideoCapture> pleft_video_capture_;
        std::shared_ptr<cv::VideoCapture> pright_video_capture_;
        
        // The image_transport based image publishers and the image_transport object
	    std::shared_ptr<image_transport::ImageTransport> pimage_transport_;

        // The main loop thread
        std::unique_ptr<std::thread> pmain_loop_thread_;

        // The zed calibrations in the conf file
        std::shared_ptr<CameraCalibrationEntry> left_2K, left_FHD, left_HD, left_VGA;
        std::shared_ptr<CameraCalibrationEntry> right_2K, right_FHD, right_HD, right_VGA;
        
        // Simulation time
        ros::Time simulation_time_;

        // ROS public node handle
        ros::NodeHandle nh_;  
        // ROS private node handle
        ros::NodeHandle nhp_;  

        // The main loop function
        void MainLoop();

        // Load calibration from Zed file
        bool LoadZedCalibration();

        // 
        sensor_msgs::ImagePtr CreateImageMsg(
            const cv::Mat_<uchar>& image, 
            const ros::Time& stamp);
	
        
    };

    // Non-blocking getch
    int nbgetch();  
}

#endif