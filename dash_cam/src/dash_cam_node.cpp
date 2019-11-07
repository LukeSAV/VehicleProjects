#include <ros/ros.h>
#include <iostream>
#include <wiringPi.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <ctime>
#include <string>
#include <std_msgs/String.h>

int main(int argc, char** argv) 
{
	/* ROS setup */
	ros::init(argc, argv, "dash_cam_node");
	ros::NodeHandle nh;

	/* Handle GPIO setup */
	wiringPiSetup();
	pinMode(0, INPUT);
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pinMode(13, OUTPUT);
	pinMode(14, OUTPUT);
	digitalWrite(13, HIGH);
	digitalWrite(14, LOW);

	/* Video setup functions */
	cv::VideoCapture cap(0);
	cv::VideoWriter output_video;
	if(!cap.isOpened()) 
	{
		return -1;
	}
	ros::Publisher status_pub = nh.advertise<std_msgs::String>("dash_cam_status", 1000);
	cv::Mat edges;
	cv::namedWindow("edges", 1);
	int ex = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
	cv::Size s = cv::Size((int) cap.get(cv::CAP_PROP_FRAME_WIDTH),
										(int) cap.get(cv::CAP_PROP_FRAME_HEIGHT));

	std::cout << "Video Resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
	std::cout << "FPS:" << cap.get(cv::CAP_PROP_FPS) << std::endl;;



	cv::Mat frame;
	int key_left, key_middle, key_right; // Pulled up to 3.3V -> 1 indicates that it hasn't been pressed, 0 that it has been pressed
	int key_left_prev, key_middle_prev, key_right_prev;
	bool is_recording = false;
	std::string video_title;

	/* Main loop */
	while(ros::ok())
	{
		std_msgs::String msg;
		msg.data = "Testing";
		status_pub.publish(msg);
		key_left = digitalRead(0);	
		key_middle = digitalRead(3);	
		key_right = digitalRead(2);	
		if(key_left == 0 && key_left_prev == 1) 
		{
			if(!is_recording) 
			{
				/* Get the current time for video title*/
				std::chrono::time_point<std::chrono::system_clock> cur_time = std::chrono::system_clock::now();
				std::time_t now_c = std::chrono::system_clock::to_time_t(cur_time);
				auto lt = std::localtime(&now_c);
				video_title =  std::to_string(lt->tm_mday) + "-" + std::to_string(lt->tm_mon) + "-" + std::to_string(lt->tm_year + 1900) + "-" + std::to_string(lt->tm_hour) + "-" + std::to_string(lt->tm_min) + "-" + std::to_string(lt->tm_sec);

				output_video.open("/media/ubuntu/500 GB Hard Disk/" + video_title + ".avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), cap.get(cv::CAP_PROP_FPS), s, true);
				if(!output_video.isOpened())
				{
					std::cerr << "Couldn't open video to write" << std::endl;
					is_recording = false;
					digitalWrite(14, LOW);

				}
				else
				{
					std::cout << "Successfully opened video file to write: " << video_title << std::endl;
					is_recording = true;	
					digitalWrite(14, HIGH);
				}
			}
			else 
			{
				output_video.release();	
				std::cout << "Closing current video being written to: " << video_title << std::endl;
				is_recording = false;
				digitalWrite(14, LOW);
			}
		}
		if(is_recording) 
		{
			cap >> frame;
			cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
			cv::GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
			cv::Canny(edges, edges, 0, 30, 3);
			cv::imshow("edges", frame);
			if(cv::waitKey(30) >= 0) break;
			ros::Duration(0.0166).sleep(); // Sleep 60 times per second
			output_video << frame;
		}
		key_left_prev = key_left;
		key_middle_prev = key_middle;
		key_right_prev = key_right;
	}
	cap.release();
	return 0;	
}
