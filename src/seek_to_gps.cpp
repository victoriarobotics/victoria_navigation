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

#include <cassert>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/seek_to_gps.h"
#include "victoria_perception/ObjectDetector.h"
#include "victoria_navigation/strategy_fn.h"

SeekToGps::SeekToGps() :
	count_Fix_msgs_received_(0),
	count_ObjectDetector_msgs_received_(0),
	count_Odometry_msgs_received_(0),
	goal_yaw_degrees_delta_threshold_(3.0),
	index_next_point_to_seek_(0),
	odometry_capturered_(false),
	state_(kSETUP)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~fix_topic_name", fix_topic_name_));
	assert(ros::param::get("~imu_topic_name", imu_topic_name_));
	assert(ros::param::get("~magnetic_declination", magnetic_declination_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	assert(ros::param::get("~use_imu", use_imu_));
	assert(ros::param::get("~waypoint_yaml_path", waypoint_yaml_path_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	assert(cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &SeekToGps::coneDetectorCb, this));
	assert(fix_sub_ = nh_.subscribe(fix_topic_name_, 1, &SeekToGps::fixCb, this));
	if (use_imu_) {
		assert(imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &SeekToGps::imuCb, this));
	}

	assert(odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &SeekToGps::odometryCb, this));

	ROS_INFO("[SeekToGps] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM fix_topic_name_: %s", fix_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM imu_topic_name_: %s", imu_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM magnetic_declination_: %7.4f", magnetic_declination_);
	ROS_INFO("[SeekToGps] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM use_imu: %s", use_imu_ ? "TRUE" : "FALSE");
	ROS_INFO("[SeekToGps] PARAM waypoint_yaml_path: %s", waypoint_yaml_path_.c_str());

	waypoints_ = YAML::LoadFile(waypoint_yaml_path_);
	if (waypoints_["gps_points"].Type() != YAML::NodeType::Sequence) {
		ROS_ERROR("[SeekToGps] Unable to load yaml file: %s", waypoint_yaml_path_.c_str());
	} else {
		ROS_INFO("[SeekToGps] Number of GPS points in the set: %ld", waypoints_["gps_points"].size());
	}
}

double SeekToGps::normalizeEuler(double yaw) {
	double result = yaw;
	while (result > 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

void SeekToGps::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_ObjectDetector_msg_ = *msg;
	count_ObjectDetector_msgs_received_++;
}

void SeekToGps::fixCb(const sensor_msgs::NavSatFixConstPtr& msg) {
	last_Fix_msg_ = *msg;
	count_Fix_msgs_received_++;
}

void SeekToGps::imuCb(const sensor_msgs::ImuConstPtr& msg) {
	last_Imu_msg_ = *msg;
	count_Imu_msgs_received_++;
}

void SeekToGps::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_Odometry_msg_ = *msg;
	count_Odometry_msgs_received_++;	
}

void SeekToGps::resetGoal() {
	odometry_capturered_ = false;
	index_next_point_to_seek_ = 0;
	state_ = kSEEKING_POINT;
}

StrategyFn::RESULT_T SeekToGps::tick() {
	geometry_msgs::Twist		cmd_vel;
	RESULT_T 					result = FATAL;
	ostringstream 				ss;
	if (StrategyFn::currentGoalName() != goalName()) {
		return FAILED;
	}

	int point_to_seek = stol(StrategyFn::currentGoalParam());
	{
		//###
		const GPS_POINT& gps_point = currentGpsPoint();
		ROS_INFO_COND(do_debug_strategy_,
					  "[SeekToGps::tick] Seeking "
					  "lat: %11.7f "
					  ", lon: %11.7f"
					  ", distance: %7.4f"
					  ", heading: %7.4f"
					  ", x: %11.7f"
					  ", y: %11.7f"
					  ", has_cone: %s",
					  gps_point.latitude,
					  gps_point.longitude,
					  gps_point.distance,
					  gps_point.bearing,
					  gps_point.x,
					  gps_point.y,
					  gps_point.has_cone ? "TRUE" : "FALSE"
					  );
		popGoal();
		popGpsPoint();
		resetGoal();
		return setGoalResult(SUCCESS)
		;
	}

	if (point_to_seek >= waypoints_["gps_points"].size()) {
		ss << " FAILED, requested point number: "
		   << point_to_seek
		   << " is greater or equal to number of points: "
		   << waypoints_["gps_points"].size();
			publishStrategyProgress("SeekToGps::tick", ss.str());
			return setGoalResult(FATAL);
	}

	if (count_ObjectDetector_msgs_received_ <= 0) {
		ss << " FAILED, no ConeDetector messages received";
		publishStrategyProgress("SeekToGps::tick", ss.str());
		return setGoalResult(FAILED); // No data yet.
	}

	if (count_Odometry_msgs_received_ <= 0) {
		ss << " FAILED, no Odometry messages received";
		publishStrategyProgress("SeekToGps::tick", ss.str());
		return setGoalResult(FAILED); // No data yet.
	}

	if (count_Fix_msgs_received_ <= 0) {
		ss << " FAILED, no Fix messages received";
		publishStrategyProgress("SeekToGps::tick", ss.str());
		return setGoalResult(FAILED); // No data yet.
	}

	if (use_imu_ && (count_Imu_msgs_received_ <= 0)) {
		ss << " FAILED, no Imu messages received";
		publishStrategyProgress("SeekToGps::tick", ss.str());
		return setGoalResult(FAILED); // No data yet.
	}

	if (false && last_ObjectDetector_msg_.object_detected) { //#####
		resetGoal();
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);
		ss << " SUCCESS Object detected, STOP";
		ss << ", area: " << last_ObjectDetector_msg_.object_area;
		publishStrategyProgress("SeekToGps::tick", ss.str());
		popGoal();
		return setGoalResult(SUCCESS); // Cone found.
	}

	// We have begun to receive ConeDetector messages but have not yet seen the cone.
	// Strategy: capture the current heading and begin a slow rotate to see if the
	// cone is detected in at most one revolution. If not, fail. If so, succeed.
	//TODO: This relies on receiving an Odometry message recently. No check is made
	// for freshness of the message--it's possible the Odometry information is from
	// a long time ago and no longer relevant. Also, it may be the robot isn't
	// publishing Odometry message. An alternate strategy would be to just begin rotation
	// and, knowing how fast the robot should be rotating, to stop when the robot should
	// have completed at least one 360 degree rotation.

	YAML::Node point = waypoints_["gps_points"][point_to_seek];
	double latitude = point["latitude"].as<double>();
	double longitude = point["longitude"].as<double>();
	bool has_cone = point["has_cone"].as<bool>();
	ROS_INFO("[SeekToGps::tick] seeking to point: %d, lat: %11.7f, lon: %11.7f, has_cone: %s",
		point_to_seek,
		latitude,
		longitude,
		has_cone ? "TRUE" : "FALSE"
		);

	sensor_msgs::NavSatFix point_as_fix;
	point_as_fix.latitude = latitude;
	point_as_fix.longitude = longitude;
	double point_bearing_gps_degrees = bearing(last_Fix_msg_, point_as_fix);
	double point_distance = distance(last_Fix_msg_, point_as_fix);
	ROS_INFO("[SeekToGps::tick] from current lat: %11.7f, lon: %11.7f, bearing: %7.4f, distance: %7.4f",
		last_Fix_msg_.latitude,
		last_Fix_msg_.longitude,
		point_bearing_gps_degrees,
		point_distance);

	goal_yaw_degrees_ = normalizeEuler(90 - point_bearing_gps_degrees);;
	double goal_yaw_degrees_delta = 0;
	double corrected_degrees = 0;

	if (use_imu_) {
		tf::Quaternion imu_orientation;
		tf::quaternionMsgToTF(last_Imu_msg_.orientation, imu_orientation);
		double imu_yaw_degrees = normalizeEuler(tf::getYaw(imu_orientation) * 360.0 / (2 * M_PI));
		imu_yaw_degrees = normalizeEuler(imu_yaw_degrees + 90); //#####
		corrected_degrees = imu_yaw_degrees + magnetic_declination_;
		ROS_INFO_COND(do_debug_strategy_, 
					  "[SeekToGps::tick] imu_yaw_degrees: %7.4f"
					  ", corrected_degrees: %7.4f",
					  imu_yaw_degrees,
					  corrected_degrees
					  );
	} else {
		tf::Quaternion odom_orientation;
		tf::quaternionMsgToTF(last_Odometry_msg_.pose.pose.orientation, odom_orientation);
		double odometry_yaw_degrees = normalizeEuler(tf::getYaw(odom_orientation) * 360.0 / (2 * M_PI));
		corrected_degrees = odometry_yaw_degrees + magnetic_declination_;
		ROS_INFO_COND(do_debug_strategy_, 
					  "[SeekToGps::tick] odometry_yaw_degrees: %7.4f"
					  ", corrected_degrees: %7.4f",
					  odometry_yaw_degrees,
					  corrected_degrees
					  );
	}

	goal_yaw_degrees_delta = goal_yaw_degrees_ - corrected_degrees;
	ROS_INFO_COND(do_debug_strategy_,
				  "[SeekToGps::tick] goal_yaw_degrees_: %7.4f, goal_yaw_degrees_delta: %7.4f", 
				  goal_yaw_degrees_, 
				  goal_yaw_degrees_delta);
	if (state_ == kSETUP) {
		state_ = kROTATING_TO_DISCOVER;
		result = RUNNING;
	} else if (state_ == kROTATING_TO_DISCOVER) {
		if (abs(goal_yaw_degrees_delta) < goal_yaw_degrees_delta_threshold_) {
			// Close enough to desired heading. Just move.
			ss << " In state kROTATING_TO_DISCOVER and delta yaw: "
			   << goal_yaw_degrees_delta
			   << " is close enough. Changing state to kSEEKING_POINT";
			state_ = kSEEKING_POINT;
			result = RUNNING;
		} else {
			// Need to rotate towards target.
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = goal_yaw_degrees_delta > 0 ? 0.4 : -0.4; //TODO ### Make configurable.
			cmd_vel_pub_.publish(cmd_vel);

			ss << " Rotating";
			ss << ", goal_yaw_degrees_: " << std::setprecision(5) << goal_yaw_degrees_;
			ss << ", corrected_degrees: " << std::setprecision(5) << corrected_degrees;
			result = RUNNING;
		}
	} else if (state_ = kSEEKING_POINT) {
		if (abs(goal_yaw_degrees_delta) >= goal_yaw_degrees_delta_threshold_) {
			ss << " In state kSEEKING_POINT and delta_yaw: "
			   << goal_yaw_degrees_delta
			   << " has become large. Changing state to kROTATING_TO_DISCOVER";
			state_ = kROTATING_TO_DISCOVER;
			result = RUNNING;
		}

		result = FATAL; //#####
	}

	publishStrategyProgress("SeekToGps::tick", ss.str());
	return setGoalResult(result);
}

SeekToGps& SeekToGps::singleton() {
    static SeekToGps singleton_;
    return singleton_;
}
