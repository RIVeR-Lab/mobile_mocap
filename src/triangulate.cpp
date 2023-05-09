// Authored by Gary Lvov and Mark Zolotas

#include "mobile_mocap/triangulate.hpp"

Triangulate::Triangulate() : Node("triangulate") {
	this->declare_parameter("number_cameras", 2);
	this->declare_parameter("marker_radius", 0.0095);

	this->num_cams = this->get_parameter("number_cameras").as_int();
	this->marker_radius = this->get_parameter("marker_radius").as_double();

	for (int i = 0; i < 3; i++) {
		this->camera_names.push_back("camera" + std::to_string(i));
		std::string topic_name = "/" + camera_names[i] + "/pixel_markers";

		this->markers2d.push_back(std::vector<cv::Point3d>());

		if (i < this->num_cams) {
			this->fetch_parameters(i);
		}

		if (i < this->num_cams) {
			this->markers_subs_.push_back(std::make_shared<message_filters::Subscriber<Markers>>(
			    this, topic_name, rclcpp::QoS(rclcpp::SensorDataQoS()).get_rmw_qos_profile()));
		} else {
			this->markers_subs_.push_back(std::make_shared<message_filters::Subscriber<Markers>>(
			    this, "/" + this->camera_names[0] + "/pixel_markers",
			    rclcpp::QoS(rclcpp::SensorDataQoS()).get_rmw_qos_profile()));
		}
	}

	this->marker_synchronizer_ = std::make_shared<message_filters::Synchronizer<approx_policy>>(
	    approx_policy(10), *this->markers_subs_[0], *this->markers_subs_[1], *this->markers_subs_[1]);
	this->marker_synchronizer_->setMaxIntervalDuration(
	    rclcpp::Duration(0, 33330000));  // 30 hz disynchronization
	this->marker_synchronizer_->registerCallback(std::bind(&Triangulate::markers_sync_callback, this,
	                                                       std::placeholders::_1, std::placeholders::_2,
	                                                       std::placeholders::_3));

	this->triangulated_markers_pub_ = this->create_publisher<mobile_mocap::msg::TriangulatedMarkers>(
	    "triangulated_markers", rclcpp::QoS(rclcpp::SensorDataQoS()));

	this->marker_array_pub_ =
	    this->create_publisher<visualization_msgs::msg::MarkerArray>("markersfused3d/debug", 10);
}

std::vector<RelativeGeometry> Triangulate::describe_geometry(const std::vector<cv::Point3d>& markers) const {
	std::vector<RelativeGeometry> marker_geometries;

	for (cv::Point3d origin : markers) {
		cv::Point2d origin_2d(origin.x, origin.y);
		RelativeGeometry marker_geo = RelativeGeometry(origin_2d);

		for (cv::Point3d other : markers) {
			if (origin.x || other.x || origin.y != other.y || origin.z != other.z) {
				double x_diff = other.x - origin.x;
				double y_diff = other.y - origin.y;
				double dist = std::sqrt((std::pow(x_diff, 2) + std::pow(y_diff, 2)));
				double angle = std::atan2(y_diff, x_diff);
				angle = fmod(angle, 2 * M_PI);  // may not be needed

				if (angle < 0) {  // normalize to be positive
					angle += (2 * M_PI);
				}

				marker_geo.relatives.push_back(std::make_tuple(dist, angle, other.z));
			}
		}
		marker_geometries.push_back(marker_geo);
	}
	return marker_geometries;
}

void Triangulate::break_close_distances_by_angle(
    std::vector<std::tuple<double, double, double>>& geometries) const {
	int pix_thresh = 10;  // potentially make this a class variable loaded from a ros2 parameter
	for (int i = 0; i < (int)geometries.size() - 1; i++) {
		if (abs(std::get<0>(geometries[i]) - std::get<0>(geometries[i + 1])) < pix_thresh) {
			if (std::get<1>(geometries[i]) > std::get<1>(geometries[i + 1])) {
				std::tuple<double, double, double> temp = geometries[i];
				geometries[i] = geometries[i + 1];
				geometries[i + 1] = temp;
			}
		}
	}
}

void Triangulate::make_geometries_consistent(std::vector<RelativeGeometry>& geom) const {
	for (RelativeGeometry& geo : geom) {
		// sort first by distance (the first element)
		std::sort(geo.relatives.begin(), geo.relatives.end(),
		          [](const std::tuple<double, double, double>& left,
		             const std::tuple<double, double, double>& right) {
			          return std::get<0>(left) < std::get<0>(right);
		          });

		// then break close distance by their angle
		this->break_close_distances_by_angle(geo.relatives);
	}
}

Triangulate::AssociatedPoints2d Triangulate::correspond_markers(const std::vector<cv::Point3d>& markers0,
                                                                const std::vector<cv::Point3d>& markers1) {
	// TODO: replace geometric matching with epipolar matching
	std::vector<RelativeGeometry> first_geometries = this->describe_geometry(markers0);
	std::vector<RelativeGeometry> second_geometries = this->describe_geometry(markers1);

	this->make_geometries_consistent(first_geometries);
	this->make_geometries_consistent(second_geometries);

	// --------------------------(Point 1, Point 2, error)
	std::vector<std::tuple<cv::Point2d, cv::Point2d, double>> potential_assignments;

	for (RelativeGeometry first_geom : first_geometries) {
		std::tuple<cv::Point2d, cv::Point2d, double> pot_assign =
		    std::make_tuple(cv::Point2d(), cv::Point2d(), DBL_MAX);

		for (RelativeGeometry second_geom : second_geometries) {
			int min_length = std::min(first_geom.relatives.size(), second_geom.relatives.size());
			double pot_assign_error = 0;

			std::vector<std::tuple<double, double, double>> descriptor_1 = first_geom.relatives;
			std::vector<std::tuple<double, double, double>> descriptor_2 = second_geom.relatives;
			for (int i = 0; i < min_length; i++) {
				double x1 = std::get<0>(descriptor_1[i]) * cos(std::get<1>(descriptor_1[i]));
				double y1 = std::get<0>(descriptor_1[i]) * sin(std::get<1>(descriptor_1[i]));
				double x2 = std::get<0>(descriptor_2[i]) * cos(std::get<1>(descriptor_2[i]));
				double y2 = std::get<0>(descriptor_2[i]) * sin(std::get<1>(descriptor_2[i]));

				double error = std::sqrt(std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2));
				double radius_error_weight = 0;
				double radius_error = abs(std::get<2>(descriptor_1[i]) - std::get<2>(descriptor_2[i]));
				radius_error *= radius_error_weight;
				pot_assign_error += (error + radius_error);
			}
			if (pot_assign_error < std::get<2>(pot_assign)) {
				pot_assign = std::make_tuple(first_geom.origin, second_geom.origin, pot_assign_error);
			}
		}
		potential_assignments.push_back(pot_assign);
	}

	std::sort(potential_assignments.begin(), potential_assignments.end(),
	          [](const std::tuple<cv::Point2d, cv::Point2d, double>& left,
	             const std::tuple<cv::Point2d, cv::Point2d, double>& right) {
		          return std::get<2>(left) < std::get<2>(right);
	          });

	std::vector<cv::Point2d> associated_markers0;
	std::vector<cv::Point2d> associated_markers1;

	std::unordered_set<std::string> assigned_first;
	std::unordered_set<std::string> assigned_second;

	for (std::tuple<cv::Point2d, cv::Point2d, double> pot_assoc : potential_assignments) {
		cv::Point2d first_pix = std::get<0>(pot_assoc);
		cv::Point2d second_pix = std::get<1>(pot_assoc);

		if (assigned_first.find(hash(first_pix)) == assigned_first.end() &&
		    assigned_second.find(hash(second_pix)) == assigned_second.end()) {  // not found in the set
			assigned_first.insert(hash(first_pix));
			assigned_second.insert(hash(second_pix));

			associated_markers0.push_back(first_pix);
			associated_markers1.push_back(second_pix);
		}
	}
	return std::make_pair(associated_markers0, associated_markers1);
}

bool Triangulate::find_ls_solution(const cv::Mat& A, const cv::Mat& b, int idx, std::pair<double, double>& d,
                                   cv::Mat& X) {
	// Solve for x
	cv::solve(A, b, X(cv::Range(0, 3), cv::Range(idx, idx + 1)), cv::DECOMP_SVD);

	// Update depths
	cv::Mat xcolT = X.col(idx).t();
	double d1_new = this->camera_data[0].proj.row(2).dot(xcolT);
	double d2_new = this->camera_data[1].proj.row(2).dot(xcolT);

	if ((std::abs(d1_new - d.first) <= DEPTH_CONVERGENCE_TOLERANCE) &&
	    (std::abs(d2_new - d.second) <= DEPTH_CONVERGENCE_TOLERANCE)) {
		return false;
	}

	// Re-weight A matrix and b vector with the new depths
	cv::Mat Ad1 = A.rowRange(0, 2) * 1 / d1_new;
	cv::Mat Ad2 = A.rowRange(2, 4) * 1 / d2_new;
	cv::Mat bd1 = b.rowRange(0, 2) * 1 / d1_new;
	cv::Mat bd2 = b.rowRange(2, 4) * 1 / d2_new;

	Ad1.copyTo(A.rowRange(0, 2));
	Ad2.copyTo(A.rowRange(2, 4));
	bd1.copyTo(b.rowRange(0, 2));
	bd2.copyTo(b.rowRange(2, 4));

	d.first = d1_new;
	d.second = d2_new;

	return true;
}

std::vector<cv::Point3d> Triangulate::iterative_least_squares_triangulation(
    const Triangulate::AssociatedPoints2d& associated_points) {
	// Initialize C matrices
	cv::Mat C1 = -cv::Mat::eye(2, 3, CV_64F);
	cv::Mat C2 = -cv::Mat::eye(2, 3, CV_64F);
	// Initialize/extract other linear algebra components
	std::vector<cv::Point2d> u1 = associated_points.first;
	std::vector<cv::Point2d> u2 = associated_points.second;
	int assoc_pts_len = u1.size();

	cv::Mat A = cv::Mat::zeros(4, 3, CV_64F);
	cv::Mat b = cv::Mat::zeros(4, 1, CV_64F);
	cv::Mat X = cv::Mat::eye(4, assoc_pts_len, CV_64F);
	cv::Mat onesRow = cv::Mat::ones(1, assoc_pts_len, CV_64F);
	onesRow.copyTo(X.row(3));

	for (int i = 0; i < assoc_pts_len; i++) {
		// Build C matrices to help construct A and b
		cv::Mat U1i = cv::Mat(u1[i]);
		cv::Mat U2i = cv::Mat(u2[i]);
		U1i.copyTo(C1.col(2));
		U2i.copyTo(C2.col(2));

		// Build A
		cv::Mat C1R1 = C1 * this->camera_data[0].proj(cv::Range(0, 3), cv::Range(0, 3));
		cv::Mat C2R2 = C2 * this->camera_data[1].proj(cv::Range(0, 3), cv::Range(0, 3));
		C1R1.copyTo(A.rowRange(0, 2));
		C2R2.copyTo(A.rowRange(2, 4));

		// Build b
		cv::Mat C1t1 = C1 * this->camera_data[0].proj(cv::Range(0, 3), cv::Range(3, 4));
		cv::Mat C2t2 = C2 * this->camera_data[1].proj(cv::Range(0, 3), cv::Range(3, 4));
		C1t1.copyTo(b.rowRange(0, 2));
		C2t2.copyTo(b.rowRange(2, 4));
		b = -1.0 * b;

		// Init depths
		std::pair<double, double> d = std::make_pair(1.0, 1.0);

		// Hartley suggests 10 iterations at most
		int num_iters = 0;
		bool not_converged = true;
		while (num_iters < MAX_LS_ITER && not_converged) {
			not_converged = this->find_ls_solution(A, b, i, d, X);
			num_iters++;
		}
	}

	std::vector<cv::Point3d> output;
	cv::Mat xT = X.rowRange(0, 3).t();
	for (int i = 0; i < assoc_pts_len; i++) {
		cv::Point3d pt(xT.row(i));
		output.push_back(pt);
	}

	return output;
}

std::vector<cv::Point3d> Triangulate::duo_triangulate(const std::vector<cv::Point3d>& marker_list2d_0,
                                                      const std::vector<cv::Point3d>& marker_list2d_1) {
	Triangulate::AssociatedPoints2d associated_markers =
	    this->correspond_markers(marker_list2d_0, marker_list2d_1);
	std::vector<cv::Point3d> triangulated_markers;


	if (associated_markers.first.size() >= 1) {
		triangulated_markers = this->iterative_least_squares_triangulation(associated_markers);
	}
	return triangulated_markers;
}

std::vector<cv::Point3d> Triangulate::markers_callback(const Markers::ConstSharedPtr& msg) {
	std::vector<cv::Point3d> markers;
	int num_markers = msg->xs.size();

	for (int i = 0; i < num_markers; i++) {
		markers.push_back(cv::Point3d(msg->xs[i], msg->ys[i], msg->rads[i]));
	}
	return markers;
}

void Triangulate::markers_sync_callback(const Markers::ConstSharedPtr& msg0,
                                        const Markers::ConstSharedPtr& msg1,
                                        const Markers::ConstSharedPtr& msg2) {
	std::vector<cv::Point3d> triangulated_markers;

	if (this->num_cams == 1) {
		std::vector<cv::Point3d> markers = this->markers_callback(msg0);
		// NOT YET IMPLEMENTED
	}

	if (this->num_cams == 2) {
		std::vector<cv::Point3d> first_set = this->markers_callback(msg0);
		std::vector<cv::Point3d> second_set = this->markers_callback(msg1);
		triangulated_markers = this->duo_triangulate(first_set, second_set);
	} else if (this->num_cams == 3) {
		std::vector<cv::Point3d> first_set = this->markers_callback(msg0);
		std::vector<cv::Point3d> second_set = this->markers_callback(msg1);
		std::vector<cv::Point3d> third_set = this->markers_callback(msg2);
		// NOT YET IMPLEMENTED
	}
	mobile_mocap::msg::TriangulatedMarkers markers_msg;
	markers_msg.header.stamp = this->get_clock().get()->now();

	for(cv::Point3d marker : triangulated_markers) {
		geometry_msgs::msg::Point point;
		point.x = marker.x;
		point.y = marker.y;
		point.z = marker.z;
		markers_msg.points.push_back(point);
	}

	this->triangulated_markers_pub_->publish(markers_msg);

	this->visualize_3d_markers(triangulated_markers);
}

void Triangulate::fetch_parameters(int camera_num) {
	std::string prefix = this->camera_names[camera_num];
	// this->declare_parameter(
	//     prefix + "_camera_matrix_flattened",
	//     std::vector<double>({592.6700, 0, 379.4987, 0, 592.4480, 203.7009, 0.0, 0.0, 1.0}));
	// this->declare_parameter(prefix + "_distortion_coefficients",
	//                         std::vector<double>({-0.4051, 0.1477, -0.0008, 0.0037, 0}));
	// this->declare_parameter(prefix + "_translation_extrinsic", std::vector<double>({0.0, 0.0, 0.0}));
	// this->declare_parameter(prefix + "_rotation_extrinsic",
	//                         std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));
	this->declare_parameter(
	    prefix + "_projection_matrix_flattened",
	    std::vector<double>({578.2907, 0, 382.9227, 0, 0, 578.47, 253.2159, 0, 0, 0, 1, 0}));
	// std::vector<double> mtx_flat_param =
	//     this->get_parameter(prefix + "_camera_matrix_flattened").as_double_array();
	// std::vector<double> dist_coeffs_param =
	//     this->get_parameter(prefix + "_distortion_coefficients").as_double_array();
	// std::vector<double> tvec_param = this->get_parameter(prefix + "_translation_extrinsic").as_double_array();
	// std::vector<double> rot_param = this->get_parameter(prefix + "_rotation_extrinsic").as_double_array();
	std::vector<double> proj_param =
	    this->get_parameter(prefix + "_projection_matrix_flattened").as_double_array();
	cv::Mat mtx = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat rot = cv::Mat::zeros(3, 3, CV_64F);

	// std::copy(mtx_flat_param.begin(), mtx_flat_param.end(), mtx.begin<double>());
	// std::copy(rot_param.begin(), rot_param.end(), rot.begin<double>());
	// std::copy(dist_coeffs_param.begin(), dist_coeffs_param.end(), dist_coeffs.begin<double>());
	// std::copy(tvec_param.begin(), tvec_param.end(), tvec.begin<double>());

	cv::Mat proj_mat = cv::Mat::zeros(3, 4, CV_64F);
	std::copy(proj_param.begin(), proj_param.end(), proj_mat.begin<double>());
	this->camera_data.push_back(Camera(mtx, dist_coeffs, tvec, rot, proj_mat));
}

void Triangulate::visualize_3d_markers(const std::vector<cv::Point3d>& marker_list) const {
	// Create a marker array message
	visualization_msgs::msg::MarkerArray marker_array;

	// Iterate over the markers in the vector
	for (int i = 0; i < (int)marker_list.size(); i++) {
		visualization_msgs::msg::Marker marker;
		marker.type = visualization_msgs::msg::Marker::SPHERE;
		marker.header.frame_id = "camera0";
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.id = i;
		marker.scale.x = .01;
		marker.scale.y = .01;
		marker.scale.z = .01;
		marker.color.r = 1.0;
		marker.color.g = 0;
		marker.color.b = 0;
		marker.color.a = 1.0;
		marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(1000));
		marker.pose.position.x = marker_list[i].x;
		marker.pose.position.y = marker_list[i].y;
		marker.pose.position.z = marker_list[i].z;
		marker_array.markers.push_back(marker);
	}

	if (marker_list.size() > 0) {
		marker_array_pub_->publish(marker_array);
	}
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Triangulate>());
	rclcpp::shutdown();
	return 0;
}