/**
* 
* George Terzakis, June 2021
*
* @brief Node for publishin rectified stereo images + calibration from ZED
*
*/

#include <slam_image_publisher/zed_stereo_publisher.h>

namespace slam_image_publisher 
{
    ZedStereoPublisher::ZedStereoPublisher(
        const ros::NodeHandle& nh, 
        const ros::NodeHandle& nhp,
        const std::string& left_info_topic_name,
		const std::string& left_image_topic_name, 
		const std::string& right_info_topic_name,
		const std::string& right_image_topic_name):
        nh_(nh), 
		nhp_(nhp),
		left_info_topic_name_(left_info_topic_name),
		left_image_topic_name_(left_image_topic_name),
		right_info_topic_name_(right_info_topic_name),
		right_image_topic_name_(right_image_topic_name),
		pmain_loop_thread_(nullptr),
		simulation_time_(ros::Time::now())                                          
    {
        if (!nhp_.getParam("left_video_filename", left_video_filename_))
		{
			ROS_ERROR("Left video filename not set! Exiting...\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("right_video_filename", right_video_filename_))
		{
			ROS_ERROR("Right video filename not set! Exiting...\n");
			ros::shutdown();
			return;
		}
		if (!nhp_.getParam("calibration_filename", calibration_filename_))
		{
			ROS_ERROR("Zed calibration filename not set! Exiting...\n");
			ros::shutdown();
			return;
		}
		// Verify that files exist
		if (!fs::exists(left_video_filename_))
		{
			ROS_ERROR("Left video file does not exist! Exiting...\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(right_video_filename_))
		{
			ROS_ERROR("Right video file does not exist! Exiting...\n");
			ros::shutdown();
			return;
		}
		if (!fs::exists(calibration_filename_))
		{
			ROS_ERROR("Calibration file does not exist! Exiting...\n");
			ros::shutdown();
			return;
		}

		// Retrieve ZED calibration from file
		if(!LoadZedCalibration())
		{
			ROS_ERROR("Could not read ZED calibration file! Exiting...\n");
			ros::shutdown();
			return;
		}

		// Try to open the video files now...
		pleft_video_capture_.reset(new cv::VideoCapture(left_video_filename_));
		if (!pleft_video_capture_->isOpened()) 
		{
			ROS_ERROR("Could not open left video file! Exiting...\n");
			ros::shutdown();
			return;
		}
		pright_video_capture_.reset(new cv::VideoCapture(right_video_filename_));
		if (!pright_video_capture_->isOpened()) 
		{
			ROS_ERROR("Could not open right video file! Exiting...\n");
			ros::shutdown();
			return;
		}
		
		
		// Initialize CameraInfo publishers
  		pleft_info_pub_ = std::make_shared<ros::Publisher>( 
			nh_.advertise<sensor_msgs::CameraInfo>(left_info_topic_name_, 5) );
		
		pright_info_pub_ = std::make_shared<ros::Publisher>( 
			nh_.advertise<sensor_msgs::CameraInfo>(right_info_topic_name_, 5) );

		// Initialize the image transport object
		pimage_transport_.reset( new image_transport::ImageTransport(nh) );
		
		// Image publishers
		pleft_image_pub_ = std::make_shared<image_transport::Publisher>( 
			pimage_transport_->advertise(left_image_topic_name_, 1) );
		
		pright_image_pub_ = std::make_shared<image_transport::Publisher>( 
			pimage_transport_->advertise(right_image_topic_name_, 1) );

		// Start the main loop
		 pmain_loop_thread_ = std::make_unique<std::thread>(&ZedStereoPublisher::MainLoop, this);
    }


    ZedStereoPublisher::~ZedStereoPublisher() 
    {
    	if (pmain_loop_thread_)
		{
			if (pmain_loop_thread_->joinable())
       		{
            	pmain_loop_thread_->join();
        	}
		}
    }

    sensor_msgs::ImagePtr ZedStereoPublisher::CreateImageMsg(
		const cv::Mat_<uchar>& opencv_image, 
		const ros::Time& stamp)
	{

		// Create ROS image messages
		std_msgs::Header header;
		header.stamp = stamp;
		sensor_msgs::ImagePtr pimage_msg = cv_bridge::CvImage(
			header, 
			"mono8", 
			opencv_image).toImageMsg();
		
		return pimage_msg;
	}

	void ZedStereoPublisher::MainLoop() 
	{
		std::cout << " starting main loop ....\n";
		// resizing to 640 x 480
		const int small_frame_rows = 480;
		const int small_frame_cols = 640;
		
		// Obtain the frame rate
		const int fps = pleft_video_capture_->get(cv::CAP_PROP_FPS);
		// Create the delay rate
		ros::Rate rate(fps);
		// Start simulation time
		simulation_time_ = ros::Time(0);
		uint32_t seq_id = 0;
		bool stopped = false;
		while (ros::ok())
		{
			// Grab frames
			cv::Mat left_frame, right_frame; // original size rgb cache
			// resized grayscale
			cv::Mat_<uchar> left_gray_small(small_frame_rows, small_frame_cols), 
							right_gray_small(small_frame_rows, small_frame_cols); 
			pleft_video_capture_->grab();
			pright_video_capture_->grab();
			pleft_video_capture_->retrieve(left_frame);
			pright_video_capture_->retrieve(right_frame);
			const double H = left_frame.rows;
			const double W = left_frame.cols;
			// Scale factors
			const double scale_x = small_frame_cols / (1.0*left_frame.cols);
			const double scale_y = small_frame_rows / (1.0*left_frame.rows);
			
			// Convert and resize to 640x480
			cv::Mat_<uchar> left_gray, right_gray;
			cv::cvtColor(left_frame, left_gray, cv::COLOR_BGR2GRAY);
			cv::cvtColor(right_frame, right_gray, cv::COLOR_BGR2GRAY);
			
			// and resize here...
			cv::resize(left_gray, left_gray_small, left_gray_small.size());
			cv::resize(right_gray, right_gray_small, right_gray_small.size());
			
			// Create camera info messages
			sensor_msgs::CameraInfo left_info;
			left_info.header.stamp = simulation_time_;
			left_info.header.frame_id = "left_cam";
			left_info.header.seq = seq_id;
			left_info.binning_x = left_info.binning_y = 0;
			left_info.width = small_frame_cols;
			left_info.height = small_frame_rows;
			left_info.distortion_model = "plumb_bob";
			const double HD_scaler_to_img_x = left_frame.cols / W;
			const double HD_scaler_to_img_y = left_frame.rows / H;
			const double scaler_x = scale_x * HD_scaler_to_img_x;
			const double scaler_y = scale_y * HD_scaler_to_img_y;
			const double lFX = left_HD->fx * scaler_x;
			const double lCX = left_HD->cx * scaler_x;
			const double lFY = left_HD->fy * scaler_y;
			const double lCY = left_HD->cy * scaler_y;
			left_info.K = {lFX, 0, lCX,
							0, lFY, lCY,
							0, 0, 1};
			
			left_info.D = {0, 0, 0, 0, 0};
			//left_info.D = {left_HD->k1, left_HD->k2, left_HD->k3, left_HD->p1, left_HD->p2};

			left_info.P = {lFX, 0, lCX, 0,
						    0,  lFY, lCY, 0,
			                0,   0,  1, 0};
			left_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};

			pleft_info_pub_->publish(left_info);

			sensor_msgs::CameraInfo right_info;
			right_info.header.stamp = simulation_time_;
			right_info.header.frame_id = "right_cam";
			right_info.header.seq = seq_id;
			right_info.binning_x = right_info.binning_y = 0;
			right_info.width = small_frame_cols;
			right_info.height = small_frame_rows;
			right_info.distortion_model = "plumb_bob";
			right_info.K = {right_HD->fx * scaler_x, 0, right_HD->cx * scaler_x,
							0, right_HD->fy * scaler_y, right_HD->cy * scaler_y,
							0, 0, 1};
			
			right_info.D = {0, 0, 0, 0, 0};
			//right_info.D = {right_HD->k1, right_HD->k2, right_HD->k3, right_HD->p1, right_HD->p2};


			right_info.P = {right_info.K[0], 0, right_info.K[2], 0,
						    0,  right_info.K[4], right_info.K[6], 0,
			                0,     0, 1, 0};
			right_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
			
			pright_info_pub_->publish(right_info);

			// Publish images
			sensor_msgs::ImagePtr left_img_msg = CreateImageMsg(left_gray_small, simulation_time_);
			pleft_image_pub_->publish(left_img_msg);
			sensor_msgs::ImagePtr right_img_msg = CreateImageMsg(right_gray_small, simulation_time_);
			pright_image_pub_->publish(right_img_msg);

			rate.sleep();
			simulation_time_ += ros::Duration(1.0 / fps);
			seq_id++;
			
			/*fflush(stdin);	
			int ch = nbgetch();
			if (ch == 32)
			{
				fflush(stdin);
				ROS_INFO_STREAM("Paused...\n");
				while((ch = nbgetch()) != 32 && ros::ok());
				ROS_INFO_STREAM("UN-paused...\n");
				ch = 0;
			}*/
		}
	}

	bool ZedStereoPublisher::LoadZedCalibration()
	{
		std::ifstream infile(calibration_filename_);
    
    	bool result = infile.is_open();
		if (!result) 
		{
			return false;
		}
			
    	// Read the 3 individual calibration (left-right 2K, FHD, HD )
		std::shared_ptr<CameraCalibrationEntry>* camera_calibrations[4][2] = 
		{ {&left_2K, &right_2K}, 
		  {&left_FHD, &right_FHD}, 
		  {&left_HD, &right_HD}, 
		  {&left_VGA, &right_VGA}};
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 2; j++)
			{
				CameraCalibrationEntry entry;
				
				std::string line;
				line = "";
				while(line.empty()||line[0]!='[') 
				{
					if(!std::getline(infile, line)) return false;
				}
				std::cout << line << std::endl;
			
				// fx
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				auto delimiterPos = line.find("=");
            	auto name = line.substr(0, delimiterPos);
            	auto value = line.substr(delimiterPos + 1);
            	entry.fx = atof(value.c_str());
				
				// fy
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.fy = atof(value.c_str());
				
				// cx
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.cx = atof(value.c_str());
				
				// cy
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.cy = atof(value.c_str());
				
				// k1
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.k1 = atof(value.c_str());
				
				// k2
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.k2 = atof(value.c_str());
				
				// k3
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.k3 = atof(value.c_str());
				
				// p1
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.p1 = atof(value.c_str());
				
				// p2
				line = "";
				while(line.empty()) 
				{
					if(!std::getline(infile, line)) return false;
				}
				delimiterPos = line.find("=");
            	name = line.substr(0, delimiterPos);
            	value = line.substr(delimiterPos + 1);
            	entry.p2 = atof(value.c_str());

				std::cout << " Calibration read: " << std::endl <<
				"fx = " << entry.fx << std::endl <<
				 "fy = " << entry.fy << std::endl <<
				 "cx = " << entry.cx << std::endl <<
				 "cy = " << entry.cy << std::endl <<
				 "k1 = " << entry.k1 << std::endl <<
				 "k2 = " << entry.k2 << std::endl <<
				 "k3 = " << entry.k3 << std::endl <<
				 "p1 = " << entry.p1 << std::endl <<
				 "p2 = " << entry.p2 << std::endl;

				*camera_calibrations[i][j] = std::make_shared<CameraCalibrationEntry>(entry);
			}
		}
    
    	infile.close();

		return true; 
	}
	

	/**
	 * @brief Non-blocking character reading. Useful to pause-play
	 */
	int nbgetch()
	{
  		static struct termios oldt, newt;
  		tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  		newt = oldt;
  		newt.c_lflag &= ~(ICANON);                 // disable buffering      
  		tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  		int c = getchar();  // read character (non-blocking)

  		tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  		return c;
	}

} // end namespace slam_image_publisher


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "zed_stereo_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  slam_image_publisher::ZedStereoPublisher zed_publisher(
	  nh, 
	  nhp);

  // Now spin ...
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  

  return 0;
}

