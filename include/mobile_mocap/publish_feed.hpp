// Authored by Gary Lvov

#ifndef PUBLISH_FEED_HPP
#define PUBLISH_FEED_HPP

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include <camera_info_manager/camera_info_manager.hpp>

class PublishFeed : public rclcpp::Node {
   public:
	PublishFeed();

   private:
	typedef sensor_msgs::msg::Image::SharedPtr ImageMsg;
	bool rectify;
	bool modify_cam_settings;
	bool publish_camera_info;
	cv::Mat camera_mtx;
	cv::Mat dist_coeffs;

	cv::VideoCapture capture_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

	void publish_feed();
	void fetch_parameters();
};
#endif