// Authored by Gary Lvov and Mark Zolotas

#ifndef TRIANGULATE_HPP
#define TRIANGULATE_HPP

#include <math.h>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "mobile_mocap/msg/triangulated_markers.hpp"
#include "mobile_mocap/msg/markers.hpp"

const double DEPTH_CONVERGENCE_TOLERANCE = 3e-5;
const int MAX_LS_ITER = 10;

struct RelativeGeometry {
	cv::Point2d origin;
	std::vector<std::tuple<double, double, double>> relatives;
	RelativeGeometry(const cv::Point2d& origin) : origin(origin) {}
};

class Triangulate : public rclcpp::Node {
   public:
	Triangulate();

   private:
   	typedef std::pair<std::vector<cv::Point2d>, std::vector<cv::Point2d>> AssociatedPoints2d;
	
	typedef mobile_mocap::msg::Markers Markers;
	typedef message_filters::sync_policies::ApproximateTime<Markers, Markers, Markers> approx_policy;
	struct Camera {
		cv::Mat mtx;
		cv::Mat dist_coeffs;
		cv::Mat tvec;
		cv::Mat rot;
		cv::Mat proj;

		Camera(const cv::Mat& mtx, const cv::Mat& dist_coeffs, const cv::Mat& tvec, const cv::Mat& rot, const cv::Mat& proj)
			: mtx(mtx), dist_coeffs(dist_coeffs), tvec(tvec), rot(rot), proj(proj) {}
	};

	inline std::string hash(const cv::Point2d& point) { return std::to_string(point.x) + " " + std::to_string(point.y); }

	int num_cams;
	double marker_radius;
	std::vector<std::vector<cv::Point3d>> markers2d;  // x, y, and radius
	std::vector<cv::Point3d> markers3d;               // x, y, and z
	std::vector<Camera> camera_data;
	std::vector<std::string> camera_names;

	std::vector<std::shared_ptr<message_filters::Subscriber<Markers>>> markers_subs_;
	std::shared_ptr<message_filters::Synchronizer<approx_policy>> marker_synchronizer_;
	rclcpp::Publisher<mobile_mocap::msg::TriangulatedMarkers>::SharedPtr triangulated_markers_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

	// Marker correspondence
	std::vector<RelativeGeometry> describe_geometry(const std::vector<cv::Point3d>& markers) const;
	void break_close_distances_by_angle(std::vector<std::tuple<double, double, double>>& geometries) const;
	void make_geometries_consistent(std::vector<RelativeGeometry>& geom) const;
	AssociatedPoints2d correspond_markers(const std::vector<cv::Point3d>& markers0, const std::vector<cv::Point3d>& markers1);

	// Marker triangulation
	bool find_ls_solution(const cv::Mat& A, const cv::Mat& b, int idx, std::pair<double, double>& d, cv::Mat& X);
	std::vector<cv::Point3d> iterative_least_squares_triangulation(const AssociatedPoints2d& associated_points);
	std::vector<cv::Point3d> duo_triangulate(const std::vector<cv::Point3d>& marker_list2d_0, const std::vector<cv::Point3d>& marker_list2d_1);

	// Callbacks and utils
	std::vector<cv::Point3d> markers_callback(const Markers::ConstSharedPtr& msg);
	void markers_sync_callback(const Markers::ConstSharedPtr& msg0, const Markers::ConstSharedPtr& msg1,
	                           const Markers::ConstSharedPtr& msg2);
	void fetch_parameters(int camera_num);
	void visualize_3d_markers(const std::vector<cv::Point3d>& marker_list) const;
};
#endif