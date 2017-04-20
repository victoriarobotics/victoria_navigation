// Copyright 2017 Michael Wimble

// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse
// or promote products derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>
#include <cassert>
#include <cstdlib>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_perception/ComputeKmeans.h"
#include "victoria_navigation/discover_cone.h"
#include "victoria_perception/ObjectDetector.h"

DiscoverCone::DiscoverCone() :
	cone_detected_sticky_(false),
	count_object_detector_msgs_received_(0),
	state_(CAPTURE_STATE)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~image_topic_name", image_topic_name_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	assert(ros::param::get("~yaw_turn_radians_per_sec", yaw_turn_radians_per_sec_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &DiscoverCone::coneDetectorCb, this);
	odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &DiscoverCone::odometryCb, this);

	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM image_topic_name: %s", image_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM yaw_turn_radians_per_sec: %7.4f", yaw_turn_radians_per_sec_);

    coneDetectorAnnotatorService_ = nh_.serviceClient<victoria_perception::AnnotateDetectorImage>("/cone_detector/annotate_detector_image");
    computeKmeansService_ = nh_.serviceClient<victoria_perception::ComputeKmeans>("/kmeans_service/compute_kmeans", true);
}

// Capture the lates ConeDetector information
void DiscoverCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detector_msg_ = *msg;
	cone_detected_sticky_ |= last_object_detector_msg_.object_detected;
	count_object_detector_msgs_received_++;
}

// Capture the lates Odometry information.
void DiscoverCone::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_odometry_msg_ = *msg;
	count_odometry_msgs_received_++;	
}

// Reset global state so this behavior can be used to solve the next problem.
void DiscoverCone::resetGoal() {
	state_ = CAPTURE_STATE;
}

DiscoverCone::ClusterStatistics DiscoverCone::getCurrentAFilter() {
	ClusterStatistics result;
    assert(ros::param::get("/cone_detector/alow_hue_range", result.min_hue));
    assert(ros::param::get("/cone_detector/ahigh_hue_range", result.max_hue));
    assert(ros::param::get("/cone_detector/alow_saturation_range", result.min_saturation));
    assert(ros::param::get("/cone_detector/ahigh_saturation_range", result.max_saturation));
    assert(ros::param::get("/cone_detector/alow_value_range", result.min_value));
    assert(ros::param::get("/cone_detector/ahigh_value_range", result.max_value));
    return result;
}

void DiscoverCone::setCurrentAFilter(DiscoverCone::ClusterStatistics values) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::IntParameter int_param;
	dynamic_reconfigure::Config conf;

	int_param.name="alow_hue_";
	int_param.value = values.min_hue;
	conf.ints.push_back(int_param);

	int_param.name="ahigh_hue_";
	int_param.value = values.max_hue;
	conf.ints.push_back(int_param);

	int_param.name="alow_saturation_";
	int_param.value = values.min_saturation;
	conf.ints.push_back(int_param);

	int_param.name="ahigh_saturation_";
	int_param.value = values.max_saturation;
	conf.ints.push_back(int_param);

	int_param.name="alow_value_";
	int_param.value = values.min_value;
	conf.ints.push_back(int_param);

	int_param.name="ahigh_value_";
	int_param.value = values.max_value;
	conf.ints.push_back(int_param);

	srv_req.config = conf;
	bool call_result = ros::service::call("/cone_detector/set_parameters", srv_req, srv_resp);
	if (!call_result) {
		ROS_ERROR("[DiscoverCone::setCurrentAFilter] ros::service::call failed");
	}
}

bool DiscoverCone::parseClusterStatistics(std::string cluster_statistics_string, DiscoverCone::ClusterStatistics& result) {
	ClusterStatistics cluster_statistics;
	std::vector<std::string> fields;

	boost::split(fields, cluster_statistics_string, boost::is_any_of(";"));
	if (fields.size() != 8) {
		ROS_ERROR("Unexpected cluster response, expected 8 fields but found %d", (int) fields.size());
		return false;
	} else {
		for (int field_index = 0; field_index < 8; field_index++) {
			std::vector<std::string> tuple;
			boost::split(tuple, fields[field_index], boost::is_any_of("="));
			if (tuple.size() != 2) {
				ROS_ERROR("Unexpected field response, expected 2 parts of the tuple but found %d", (int) tuple.size());
				return false;
			} else {
				// ROS_INFO("cluster: %2d, field: %1d, name: %15s, value: %s",
				// 		 cluster_index,
				// 		 field_index,
				// 		 tuple[0].c_str(),
				// 		 tuple[1].c_str());
				if ((tuple[0] == "cluster") || tuple[0] == "{cluster") {
					cluster_statistics.cluster_number = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "min_hue") {
					cluster_statistics.min_hue = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "max_hue") {
					cluster_statistics.max_hue = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "min_saturation") {
					cluster_statistics.min_saturation = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "max_saturation") {
					cluster_statistics.max_saturation = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "min_value") {
					cluster_statistics.min_value = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "max_value") {
					cluster_statistics.max_value = atoi(tuple[1].c_str());
				} else  if (tuple[0] == "pixels") {
					cluster_statistics.pixels = atoi(tuple[1].c_str());
				} else {
					ROS_ERROR("Unexpected field name: %s", tuple[0].c_str());
					return false;
				}
			}
		}

		if ((cluster_statistics.cluster_number == -1) ||
			(cluster_statistics.min_hue == -1) ||
			(cluster_statistics.max_hue == -1) ||
			(cluster_statistics.min_saturation == -1) ||
			(cluster_statistics.max_saturation == -1) ||
			(cluster_statistics.min_value == -1) ||
			(cluster_statistics.max_value == -1) ||
			(cluster_statistics.pixels == -1)) {
			ROS_ERROR("Missing a field definition");
			return false;
		} else {
			ROS_INFO("cluster: %d, min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d, pixel_count: %d",
					 cluster_statistics.cluster_number, 
					 cluster_statistics.min_hue, 
					 cluster_statistics.max_hue, 
					 cluster_statistics.min_saturation, 
					 cluster_statistics.max_saturation, 
					 cluster_statistics.min_value, 
					 cluster_statistics.max_value, 
					 cluster_statistics.pixels);
		}

		result = cluster_statistics;
		return true;
	}
}

bool DiscoverCone::isLikelyConeCluster(DiscoverCone::ClusterStatistics cluster) {
	return (cluster.min_hue <= 7) &&
		   (cluster.max_hue >= 5) &&
		   (cluster.max_hue <= 32) &&
		   (cluster.pixels >= 200);
}

DiscoverCone::ClusterStatistics DiscoverCone::computeLikelyConeParameters(const std::vector<ClusterStatistics>& clusters) {
	ClusterStatistics merged_statistics;
	merged_statistics.min_hue = 179;
	merged_statistics.max_hue = 0;
	merged_statistics.min_saturation = 255;
	merged_statistics.max_saturation = 0;
	merged_statistics.min_value = 255;
	merged_statistics.max_value = 0;
	merged_statistics.pixels = 0;
	for (ClusterStatistics cluster : clusters) {
		if (isLikelyConeCluster(cluster)) {
			ROS_INFO("[DiscoverCone::computeLikelyConeParameters] selecting cluster: %d", cluster.cluster_number);
			merged_statistics.min_hue 		 = std::min(merged_statistics.min_hue, cluster.min_hue);
			merged_statistics.max_hue 		 = std::max(merged_statistics.max_hue, cluster.max_hue);
			merged_statistics.min_saturation = std::min(merged_statistics.min_saturation, cluster.min_saturation);
			merged_statistics.max_saturation = std::max(merged_statistics.max_saturation, cluster.max_saturation);
			merged_statistics.min_value		 = std::min(merged_statistics.min_value, cluster.min_value);
			merged_statistics.max_value		 = std::max(merged_statistics.max_value, cluster.max_value);
			merged_statistics.pixels		+= cluster.pixels;
		}
	}

	return merged_statistics;
}

void DiscoverCone::setSafeConeDetectorParameters(DiscoverCone::ClusterStatistics& int_out_parameters) {
	if (int_out_parameters.min_hue < 0) int_out_parameters.min_hue = 0;
	if (int_out_parameters.max_hue > 179) int_out_parameters.max_hue = 179;
	if (int_out_parameters.min_saturation < 0) int_out_parameters.min_saturation = 0;
	if (int_out_parameters.max_saturation > 255) int_out_parameters.max_saturation = 255;
	if (int_out_parameters.min_value < 0) int_out_parameters.min_value = 0;
	if (int_out_parameters.max_value > 255) int_out_parameters.max_value = 255;
}

DiscoverCone::ClusterStatistics DiscoverCone::findNewConeDetectorParameters() {
	static const int number_of_clusters = 16;
	victoria_perception::ComputeKmeans kmeans_request; // The kmeans request.
	kmeans_request.request.attempts = 1;
	kmeans_request.request.image_topic_name = image_topic_name_;
	kmeans_request.request.number_clusters = number_of_clusters;
	kmeans_request.request.resize_width = 320;
	kmeans_request.request.show_annotated_window = false;
	computeKmeansService_.call(kmeans_request);
	while (ros::ok() && kmeans_request.response.result_msg.empty()) {
		ros::spinOnce();
		cv::waitKey(10);
	}

	std::vector<std::string> clusters;
	boost::split(clusters, kmeans_request.response.kmeans_result, boost::is_any_of("}"));
	if (clusters.size() != (number_of_clusters + 1)) {
		ROS_ERROR("Unexpected service response, expected %d clusters but found %d", (number_of_clusters + 1), (int) clusters.size());
		return ClusterStatistics();
	} else {
		// Build a list of cluster statistics.
		std::vector<ClusterStatistics> cluster_list;
		for (int cluster_index = 0; cluster_index < clusters.size(); cluster_index++) {
			if (clusters[cluster_index] == "") continue;
			ClusterStatistics cluster_statistics;
			if (!parseClusterStatistics(clusters[cluster_index], cluster_statistics)) {
				ROS_ERROR("Unable to parse kmeans result for cluster index: %d", cluster_index);
				return ClusterStatistics();
			} else {
				cluster_list.push_back(cluster_statistics);
			}
		}

		ClusterStatistics new_parameters = computeLikelyConeParameters(cluster_list);

		if (new_parameters.min_hue == 179) {
			// ### Test for now good kmeans results.
			return new_parameters;
		}

		// Fiddle with results.
		new_parameters.min_hue -= 2;
		new_parameters.max_hue += 2;
		new_parameters.min_saturation -= 4;
		new_parameters.max_saturation += 30;
		new_parameters.min_value -= 10;
		new_parameters.max_value = 255;
		setSafeConeDetectorParameters(new_parameters);
		ROS_INFO("[DiscoverCone::findNewConeDetectorParameters] new parameters:, min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d, pixel_count: %d",
				 new_parameters.min_hue, 
				 new_parameters.max_hue, 
				 new_parameters.min_saturation, 
				 new_parameters.max_saturation, 
				 new_parameters.min_value, 
				 new_parameters.max_value, 
				 new_parameters.pixels);
		return new_parameters;
	}
}

bool DiscoverCone::canFindCone() {
	ClusterStatistics new_parameters = findNewConeDetectorParameters();
	if ((new_parameters.min_hue == 0) || (new_parameters.max_hue == 149)) {
		// Unsuccessful.
		ROS_INFO("[DiscoverCone::canFindCone] failed");
		return false;
	} else {
		ClusterStatistics old_parameters = getCurrentAFilter();
		ROS_INFO("[DiscoverCone::canFindCone] previous A-filter min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d",
				 old_parameters.min_hue, 
				 old_parameters.max_hue,
				 old_parameters.min_saturation,
				 old_parameters.max_saturation,
				 old_parameters.min_value,
				 old_parameters.max_value);
		setCurrentAFilter(new_parameters);
		ROS_INFO("[DiscoverCone::canFindCone] new A-filter min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d",
				 new_parameters.min_hue, 
				 new_parameters.max_hue,
				 new_parameters.min_saturation,
				 new_parameters.max_saturation,
				 new_parameters.min_value,
				 new_parameters.max_value);
		return true;
	}
}

StrategyFn::RESULT_T DiscoverCone::tick() {
	RESULT_T 					result = FATAL;	// Assume fatality in the algorithm.
	std::ostringstream 			ss;				// For sending informations messages.

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_object_detector_msgs_received_ <= 0) {
		// Wait until ConeDetector messages are received.
	    annotator_request_.request.annotation = "UL;FFFFFF;DC Cone wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	if (count_odometry_msgs_received_ <= 0) {
		// Wait until Odometry messages are received.
	    annotator_request_.request.annotation = "UL;FFFFFF;DC Odom wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	if (last_object_detector_msg_.object_detected) {
	// Publish information.
		ss << " SUCCESS Object detected, STOP";
		ss << ", area: " << last_object_detector_msg_.object_area;
		publishStrategyProgress("DiscoverCone::tick", ss.str());
		return succeedGoal(); // Success -- cone found.
	}

	switch (state_) {
	case CAPTURE_STATE:
		result = doCaptureState(ss);
		break;

	case ROTATING_TO_DISCOVER:
		result = doRotatingToDiscover(ss);
		break;

	case RECOVER_WAIT_NEXT_CONE_DETECTOR_MESSAGE:
		result = doRecoverWaitNextConeDetectionMessage(ss);
		break;

	default:
		ss << "INVALID STATE";
		break;
	}

	publishStrategyProgress("DiscoverCone::tick", ss.str());
	return result;
}

StrategyFn::RESULT_T DiscoverCone::succeedGoal() {
	geometry_msgs::Twist cmd_vel;		// For sending movement commands to the robot.
	cmd_vel.linear.x = 0;	// Stop motion.
	cmd_vel.angular.z = 0.0;
	cmd_vel_pub_.publish(cmd_vel);

	// Standard way to indicate success.
	popGoal();
	resetGoal();
    annotator_request_.request.annotation = "UL;FFFFFF;DC cone SUCCESS";
    coneDetectorAnnotatorService_.call(annotator_request_);
	return setGoalResult(SUCCESS); // Cone found.
}

StrategyFn::RESULT_T DiscoverCone::failGoal() {
	popGoal();
	resetGoal();
    annotator_request_.request.annotation = "UL;FFFFFF;DC FAIL";
    coneDetectorAnnotatorService_.call(annotator_request_);
	return setGoalResult(FAILED);
}

StrategyFn::RESULT_T DiscoverCone::doCaptureState(std::ostringstream& out_ss) {
	starting_odometry_msg_ = last_odometry_msg_;
	previous_pose_ = last_odometry_msg_.pose.pose.orientation;
	starting_yaw_ = tf::getYaw(starting_odometry_msg_.pose.pose.orientation);
	total_rotated_yaw_ = 0;
	state_ = ROTATING_TO_DISCOVER;	// We have a starting orientation, go to the begin-rotation strategy.
	out_ss << " Got Odometry, begin rotation";
	recovery_retry_count_ = 0;
	return RUNNING;
}

double DiscoverCone::computeTotalRotatedYaw() {
	tf::Quaternion 				current_tf_quat;
	double 						current_yaw = 0.0;
	double 						original_yaw = 0.0;
	tf::Quaternion 				previous_tf_quat;

	tf::quaternionMsgToTF(previous_pose_, previous_tf_quat);
	tf::quaternionMsgToTF(last_odometry_msg_.pose.pose.orientation, current_tf_quat);
	original_yaw = tf::getYaw(starting_odometry_msg_.pose.pose.orientation);
	current_yaw = tf::getYaw(last_odometry_msg_.pose.pose.orientation);
	total_rotated_yaw_ = total_rotated_yaw_ + fabs(previous_tf_quat.angleShortestPath(current_tf_quat));
	previous_pose_ = last_odometry_msg_.pose.pose.orientation;
}

StrategyFn::RESULT_T DiscoverCone::doRotatingToDiscover(std::ostringstream& out_ss) {
	// Rotate a bit if not already completed a full circle.
	RESULT_T result = FATAL;

	computeTotalRotatedYaw();
	if (total_rotated_yaw_ > (2 * M_PI)) {
		// Completed a full rotation.
		out_ss << "ROTATION FAILED no cone found after one rotation. ";
		if (recovery_retry_count_ > 1) { //### Maybe more?
			result = failGoal();
		} else {
			// More retries are available.
			recovery_retry_count_++;
		    annotator_request_.request.annotation = "UL;FFFFFF;DC RECOVERY";
		    coneDetectorAnnotatorService_.call(annotator_request_);
		    if (canFindCone()) {
		    	// Reset data for another rotation
		    	doCaptureState(out_ss);

				// Capture current cone detector message sequence number.
				recovery_start_sequence_number_ = count_object_detector_msgs_received_;
		    	state_ = RECOVER_WAIT_NEXT_CONE_DETECTOR_MESSAGE;	// See if that helped.
		    	cone_detected_sticky_ = false; // Start looking for object now.
		    	result = RUNNING;
		    } else {
				result = failGoal();
		    }
		}
	} else {
		// Haven't completed a full rotation yet, keep rotating.
		geometry_msgs::Twist	cmd_vel;		// For sending movement commands to the robot.

		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = yaw_turn_radians_per_sec_ / 2.0;
		cmd_vel_pub_.publish(cmd_vel);

		out_ss << "Rotating";
		result = RUNNING;
	}

    annotator_request_.request.annotation = "UL;FFFFFF;DC rotating";
    coneDetectorAnnotatorService_.call(annotator_request_);
    return result;
}

StrategyFn::RESULT_T DiscoverCone::doRecoverWaitNextConeDetectionMessage(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;

	if (cone_detected_sticky_) {
		// Cone detector adjustmenst worked. Success.
		ROS_INFO("[DiscoverCone::doRecoverWaitNextConeDetectionMessage] found cone, SUCCESS");
		return succeedGoal();
	}

	if (total_rotated_yaw_ > (2 * M_PI)) {
		// Completed a full rotation.
		out_ss << "RECOVERY ROTATION FAILED no cone found after one rotation. ";
		result = failGoal();
	} else if (count_object_detector_msgs_received_ > (recovery_start_sequence_number_ + 6)) {
		// Wait until the cone detector has had a chance with the new settings.
	} else {
		// Haven't completed a full rotation yet, keep rotating.
		geometry_msgs::Twist	cmd_vel;		// For sending movement commands to the robot.

		canFindCone();
		recovery_start_sequence_number_ = count_object_detector_msgs_received_;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = yaw_turn_radians_per_sec_ / 2.0;
		cmd_vel_pub_.publish(cmd_vel);

	    annotator_request_.request.annotation = "UL;FFFFFF;DC R-rotating";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		out_ss << "RECOVERY Rotating";
		result = RUNNING;
	}

	return result;
}

StrategyFn& DiscoverCone::singleton() {
    static DiscoverCone singleton_;
    return singleton_;
}
