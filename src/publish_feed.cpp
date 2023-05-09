// Authored by Gary Lvov

#include "mobile_mocap/publish_feed.hpp"

PublishFeed::PublishFeed() : Node("publish_feeds") {
	this->fetch_parameters();  // fill this->camera_mtx and this->dist_coeffs based off parameters

	
	if (this->modify_cam_settings) {
		this->capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // lower exposure, isolate retroreflectors
		this->capture_.set(cv::CAP_PROP_EXPOSURE, 5); // TODO: replace with parameter loading from .yaml
		this->capture_.set(cv::CAP_PROP_AUTO_WB, 0);
		this->capture_.set(cv::CAP_PROP_WB_TEMPERATURE, 4000);
	}
	
	this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("feed", 5);

	double time_period = 1.0 / get_parameter("frequency").as_int();
	timer_ = this->create_wall_timer(std::chrono::duration<double>(time_period),
	                                 std::bind(&PublishFeed::publish_feed, this));
}

void PublishFeed::publish_feed() {
	cv::Mat frame;
	cv::Mat rectified_frame;
	
	
	if (this->capture_.read(frame)) {
		ImageMsg image_msg;
		if (this->rectify) {
			cv::undistort(frame, rectified_frame, camera_mtx, dist_coeffs);
			image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified_frame).toImageMsg();
		} else {
			image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
		}
		image_msg->header.stamp =  this->get_clock().get()->now();
		image_msg->header.frame_id = "camera0"; //this->get_namespace();
		if(this->publish_camera_info) {
			sensor_msgs::msg::CameraInfo info_msg;
			info_msg = this->cinfo_manager_->getCameraInfo();
			info_msg.header = image_msg->header;
			this->camera_info_pub_->publish(info_msg);
		}
		image_pub_->publish(*image_msg.get());
	}
}

void PublishFeed::fetch_parameters() {
	this->declare_parameter("rectify", false);
	this->declare_parameter("modify_cam_settings", true);
	this->declare_parameter("publish_camera_info", false);
	this->declare_parameter("port", 0);
	this->declare_parameter("frequency", 30);
	this->declare_parameter("exposure_value", 5);
	this->declare_parameter(
	    "camera_matrix_flattened",
	    std::vector<double>({592.6700, 0, 379.4987, 0, 592.4480, 203.7009, 0.0, 0.0, 1.0}));
	this->declare_parameter("distortion_coefficients",
	                        std::vector<double>({-0.414937689315460, 0.169092343208053, 0, 0, 0}));
	this->modify_cam_settings = this->get_parameter("modify_cam_settings").as_bool();
	this->rectify = this->get_parameter("rectify").as_bool();
	this->publish_camera_info = this->get_parameter("publish_camera_info").as_bool();

	std::vector<double> cam_mtx_flat_param = this->get_parameter("camera_matrix_flattened").as_double_array();
	std::vector<double> dist_coeffs_param = this->get_parameter("distortion_coefficients").as_double_array();

	this->camera_mtx = cv::Mat::zeros(3, 3, CV_64F);
	this->dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

	std::copy(cam_mtx_flat_param.begin(), cam_mtx_flat_param.end(), camera_mtx.begin<double>());
	std::copy(dist_coeffs_param.begin(), dist_coeffs_param.end(), dist_coeffs.begin<double>());

	if (this->publish_camera_info) {
		/* get ROS2 config parameter for camera calibration file */
		this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
		this->cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
		this->cinfo_manager_->loadCameraInfo(this->get_parameter("camera_calibration_file").as_string());
		this->camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 5);
	}

	this->capture_ = cv::VideoCapture(get_parameter("port").as_int(), cv::CAP_V4L2);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PublishFeed>());
	rclcpp::shutdown();
	return 0;
}