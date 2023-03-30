// Authored by Gary Lvov

#ifndef MARKER_DETECTOR_HPP
#define MARKER_DETECTOR_HPP

#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "mobile_mocap/msg/markers.hpp"

class MarkerDetector : public rclcpp::Node {
   public:
	MarkerDetector();

   private:
	typedef std::vector<std::vector<cv::Point>> Contours;
	typedef sensor_msgs::msg::Image::SharedPtr ImageMsg;

	bool debug;
	int frequency;
	int circularity_threshold;
	cv::Mat blank_debug_img;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _feed_sub;
	rclcpp::Publisher<mobile_mocap::msg::Markers>::SharedPtr _marker_pub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _debug_image_pub;

	cv_bridge::CvImagePtr feed_ptr;

	cv::Mat isolate_retroreflectors(const cv::Mat& image) const;
	void fill_in_edges(cv::Mat& edges) const;
	void find_markers(const cv::Mat& image);
	void img_callback(const ImageMsg msg);
};
#endif