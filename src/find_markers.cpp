// Authored by Gary Lvov

#include "mobile_mocap/marker_detector.hpp"

MarkerDetector::MarkerDetector() : Node("MarkerDetector") {
	this->declare_parameter("frequency", 30);
	this->declare_parameter("circularity_thresh", .6);
	this->declare_parameter("publish_debug_stream", true);

	this->frequency = this->get_parameter("frequency").as_int();
	this->circularity_threshold = this->get_parameter("circularity_thresh").as_double();
	this->debug = this->get_parameter("publish_debug_stream").as_bool();

	this->_feed_sub = this->create_subscription<sensor_msgs::msg::Image>(
	    "feed", 10, std::bind(&MarkerDetector::img_callback, this, std::placeholders::_1));

	this->_marker_pub = this->create_publisher<mobile_mocap::msg::Markers>(
	    "pixel_markers", rclcpp::QoS(rclcpp::SensorDataQoS()));

	if (debug) {
		_debug_image_pub = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
	}
}

cv::Mat MarkerDetector::isolate_retroreflectors(const cv::Mat& image) const {
	/*
	When auto white balance is disabled, light visible to us appears green in the
	near-infrared feed. We isolate the retroreflectors by removing all pixels that are dominated by green.
	Then, we threshold to return a binarized image of retroreflectors and not.
	*/
	cv::Mat filtered(image.rows, image.cols, CV_8UC3);

	for (int row = 0; row < image.rows; row++) {
		for (int col = 0; col < image.cols; col++) {
			cv::Vec3b pix = image.at<cv::Vec3b>(row, col);

			int r = pix[2];
			int g = pix[1];
			int b = pix[0];

			if (g > b || g > r) {
				pix[0] = 0;
				pix[1] = 0;
				pix[2] = 0;
			}
			filtered.at<cv::Vec3b>(row, col) = pix;
		}
	}
	cv::Mat grey;
	cv::cvtColor(filtered, grey, cv::COLOR_BGR2GRAY);

	cv::Mat blurred;
	cv::GaussianBlur(grey, blurred, cv::Size(3, 3), 0);

	cv::Mat binarized;
	cv::threshold(blurred, binarized, 40, 255, cv::THRESH_BINARY);
	return binarized;
}
/**
 * Fill in edges, removing nested contours.
 */
void MarkerDetector::fill_in_edges(cv::Mat& edges) const {
	Contours contours;
	cv::findContours(edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (std::vector<cv::Point> contour : contours) {
		for (int i = 0; i < (int)contour.size(); i++) {
			for (int j = 0; j < (int)contour.size(); j++) {
				cv::line(edges, contour[i], contour[j], cv::Scalar(255, 255, 255), 1);
			}
		}
	}
}

/**
 * Find retroreflective markers within an image
 */
void MarkerDetector::find_markers(const cv::Mat& image) {
	mobile_mocap::msg::Markers markers;
	cv::Mat binarized = this->isolate_retroreflectors(image);

	cv::Canny(binarized, binarized, 100, 200);

	// Pre-emptively try to get rid of some nested features
	this->fill_in_edges(binarized);

	Contours contours;
	cv::findContours(binarized, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (std::vector<cv::Point> contour : contours) {
		if (contour.size() > 4) {  // Must be at least 5 points to fit ellipse
			try {
				cv::RotatedRect ellipse = cv::fitEllipse(contour);
				// average the major and minor axis to approximate the radius.
				double size = (ellipse.size.width + ellipse.size.height) / 4;
				if (size > 2) {
					if (this->debug) {
						cv::ellipse(image, ellipse.center, cv::Size(1, 1), 0, 360, 0, cv::Scalar(0, 255, 0),
						            2, 8);
						cv::ellipse(image, ellipse.center, cv::Size(size, size), 0, 360, 0,
						            cv::Scalar(255, 0, 0), 2, 8);
					}

					markers.xs.push_back(ellipse.center.x);
					markers.ys.push_back(ellipse.center.y);
					markers.rads.push_back(size);
				}

			} catch (cv::Exception& e) {
				const char* err_msg = e.what();
				std::cout << "Warning, exception caught: " << err_msg;
				RCLCPP_DEBUG_STREAM(this->get_logger(), "Warning, exception caught: " << err_msg;);
			}
		}
	}

	if (this->debug) {
		ImageMsg msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
		this->_debug_image_pub->publish(*msg.get());
	}

	markers.header.stamp = this->get_clock().get()->now();
	this->_marker_pub->publish(markers);
}

void MarkerDetector::img_callback(const ImageMsg msg) {
	try {
		this->feed_ptr = cv_bridge::toCvCopy(msg);
		this->find_markers(this->feed_ptr->image);
	} catch (cv_bridge::Exception& e) {
		RCLCPP_DEBUG(this->get_logger(), "Cannot convert image to mat %d", 4);
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MarkerDetector>());
	rclcpp::shutdown();
	return 0;
}