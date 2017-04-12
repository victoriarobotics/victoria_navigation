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
	count_fix_msgs_received_(0),
	count_object_detector_msgs_received_(0),
	count_odometry_msgs_received_(0),
	goal_yaw_degrees_delta_threshold_(3.0),
	linear_move_meters_per_sec_(0.4),
	state_(SETUP),
	yaw_turn_radians_per_sec_(0.4)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~fix_topic_name", fix_topic_name_));
	assert(ros::param::get("~gps_close_distance_meters", gps_close_distance_meters_));
	assert(ros::param::get("~imu_topic_name", imu_topic_name_));
	assert(ros::param::get("~linear_move_meters_per_sec", linear_move_meters_per_sec_));
	assert(ros::param::get("~magnetic_declination", magnetic_declination_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	assert(ros::param::get("~solve_using_odom", solve_using_odom_));
	assert(ros::param::get("~use_imu", use_imu_));
	assert(ros::param::get("~yaw_turn_radians_per_sec", yaw_turn_radians_per_sec_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	assert(cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &SeekToGps::coneDetectorCb, this));
	assert(fix_sub_ = nh_.subscribe(fix_topic_name_, 1, &SeekToGps::fixCb, this));
	assert(imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &SeekToGps::imuCb, this));

	assert(odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &SeekToGps::odometryCb, this));

	ROS_INFO("[SeekToGps] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM fix_topic_name: %s", fix_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM gps_close_distance_meters: %7.4f", gps_close_distance_meters_);
	ROS_INFO("[SeekToGps] PARAM imu_topic_name: %s", imu_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM linear_move_meters_per_sec: %7.4f", linear_move_meters_per_sec_);
	ROS_INFO("[SeekToGps] PARAM magnetic_declination: %7.4f", magnetic_declination_);
	ROS_INFO("[SeekToGps] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM solve_using_odom: %s", solve_using_odom_ ? "TRUE" : "FALSE");
	ROS_INFO("[SeekToGps] PARAM use_imu: %s", use_imu_ ? "TRUE" : "FALSE");
	ROS_INFO("[SeekToGps] PARAM yaw_turn_radians_per_sec: %7.4f", yaw_turn_radians_per_sec_);
}

// Capture the lates ConeDetector information
void SeekToGps::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detector_msg_ = *msg;
	count_object_detector_msgs_received_++;
}

// Capture the lates Fix information.
void SeekToGps::fixCb(const sensor_msgs::NavSatFixConstPtr& msg) {
	last_fix_msg_ = *msg;
	count_fix_msgs_received_++;
}

// Capture the latest Imu information.
void SeekToGps::imuCb(const sensor_msgs::ImuConstPtr& msg) {
	last_imu_msg_ = *msg;
	count_imu_msgs_received_++;
}

// Capture the lates Odometry information.
void SeekToGps::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_odometry_msg_ = *msg;
	count_odometry_msgs_received_++;	
}

// Reset global state so this behavior can be used to solve the next problem.
void SeekToGps::resetGoal() {
	state_ = SEEKING_POINT;
}

double SeekToGps::odomBearing(double x1, double y1, double x2, double y2) {
    if ((x1 == x2) && (y1 == y2)) return 0;
    double theta = atan2(x2 - x1, y1 - y2);
    if (theta < 0.0)
        theta += 2 * M_PI;
    return theta;
}

StrategyFn::RESULT_T SeekToGps::tick() {
	geometry_msgs::Twist		cmd_vel;		// For sending movement commands to the robot.
	RESULT_T 					result = FATAL;	// Assume fatality in the algorithm.
	std::ostringstream 				ss;				// For sending informations messages.

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_object_detector_msgs_received_ <= 0) {
		// Wait until ConeDetector messages are received.
		ROS_INFO("[SeekToGps::tick] waiting ObjectDetector messages");
		return RUNNING;
	}

	if (count_odometry_msgs_received_ <= 0) {
		// Wait until Odometry messages are received.
		ROS_INFO("[SeekToGps::tick] waiting Odometry messages");
		return RUNNING;
	}

	if (count_fix_msgs_received_ <= 0) {
		// Wait until Fix messages are received.
		ROS_INFO("[SeekToGps::tick] waiting Fix messages");
		return RUNNING;
	}

	if (count_imu_msgs_received_ <= 0) {
		// Wait until Imu messages are received.
		ROS_INFO("[SeekToGps::tick] waiting Imu messages");
		return RUNNING;
	}

	const GPS_POINT& goal_gps_point = currentGpsPoint();	// Get the goal waypoint.

	if (goal_gps_point.has_cone && last_object_detector_msg_.object_detected) {
		// Success! Stop when a RoboMagellan cone is seen.
		resetGoal();
		
		cmd_vel.linear.x = 0;	// Stop motion.
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);

		// Publish information.
		ss << "SUCCESS Object detected, STOP";
		ss << ", area: " << last_object_detector_msg_.object_area;
		publishStrategyProgress("SeekToGps::tick", ss.str());
		
		// Standard way to indicate success.
		popGoal();
		return setGoalResult(SUCCESS);
	}

	// Convert the goal waypoint into a NavSatFix format.
	sensor_msgs::NavSatFix goal_as_nav_sat_fix;
	goal_as_nav_sat_fix.latitude = goal_gps_point.latitude;
	goal_as_nav_sat_fix.longitude = goal_gps_point.longitude;

	// Compute the heading and distance from the current position to the goal waypoint.
	double current_to_goal_bearing;
	if (solve_using_odom_) {
		current_to_goal_bearing = odomBearing(last_odometry_msg_.pose.pose.position.x, last_odometry_msg_.pose.pose.position.y,
												goal_gps_point.x, goal_gps_point.y);
	} else {
		current_to_goal_bearing = bearing(last_fix_msg_, goal_as_nav_sat_fix);
	}

	double point_distance;
	if (solve_using_odom_) {
		point_distance = sqrt(((last_odometry_msg_.pose.pose.position.x - goal_gps_point.x) * (last_odometry_msg_.pose.pose.position.x - goal_gps_point.x)) +
							  ((last_odometry_msg_.pose.pose.position.y - goal_gps_point.y) * (last_odometry_msg_.pose.pose.position.y - goal_gps_point.y)));
	} else {
		point_distance = distance(last_fix_msg_, goal_as_nav_sat_fix);
	}

	// Begin forming an informational message.
	ss << "AT lat: " << std::setprecision(10) << last_fix_msg_.latitude
	   << ", lon: " << std::setprecision(10) << last_fix_msg_.longitude
	   << ", x: " << std::setprecision(3) << last_odometry_msg_.pose.pose.position.x
	   << ", y: " << std::setprecision(3) << last_odometry_msg_.pose.pose.position.y
	   << ", dist: "  << std::setprecision(3) << point_distance
	   << ", cg_bearing/d: " << std::setprecision(4) << angles::to_degrees(current_to_goal_bearing);

	// Is the robot "close enough"?
	if (point_distance < gps_close_distance_meters_) {
		// Close enough.
		resetGoal();	// Reset the global state so the behavior can be used for another problem.

		// Stop the robot.
		// cmd_vel.linear.x = 0;
		// cmd_vel.angular.z = 0.0;
		// cmd_vel_pub_.publish(cmd_vel);

		// Publish information.
		ss << ". SUCCESS close to point, STOP";
		publishStrategyProgress("SeekToGps::tick", ss.str());

		// Standard way to indicate success.
		popGoal();
		return setGoalResult(SUCCESS);
	}

	// Compute the direction the robot needs to be pointing to.
	goal_yaw_ = angles::normalize_angle((M_PI / 2.0) - current_to_goal_bearing);; // Convert GPS heading to ROS heading.
	double goal_yaw_delta = 0; // How far off is the current heading from the desired heading?
	double corrected_yaw = 0;  // Current heading corrected, if necessary, by magnetic declination.

	if (use_imu_) {
		// Use the Imu as truth for the current robot heading.
		tf::Quaternion imu_orientation;
		tf::quaternionMsgToTF(last_imu_msg_.orientation, imu_orientation);
		double imu_yaw = tf::getYaw(imu_orientation);
		corrected_yaw = imu_yaw + angles::from_degrees(magnetic_declination_);
		ss << ", imuyaw/d: " << std::setprecision(4) << angles::to_degrees(corrected_yaw);
	} else {
		// Use the Odometry as truth for the current robot heading.
		tf::Quaternion odom_orientation;
		tf::quaternionMsgToTF(last_odometry_msg_.pose.pose.orientation, odom_orientation);
		double odometry_yaw = tf::getYaw(odom_orientation);
		corrected_yaw = odometry_yaw;
		ss << ", odoyaw/d: " << std::setprecision(4) << angles::to_degrees(odometry_yaw);
	}

	goal_yaw_delta = angles::shortest_angular_distance(corrected_yaw, goal_yaw_);  // How far off is the current heading?
	ss << ", goal/d: " << std::setprecision(4) << angles::to_degrees(goal_yaw_);
	ss << ", delta/d: " << std::setprecision(4) << angles::to_degrees(goal_yaw_delta);
	switch (state_) {
	case SETUP:
		// The first time through, just gather data.
		// Set up so that the next time through, the robot is rotated to point to the goal waypoint.
		ss << ". Setting up"
		   << ", TO lat: " << std::setprecision(10) << goal_gps_point.latitude
		   << ", lon: " << std::setprecision(10) << goal_gps_point.longitude
		   << ", head: " << std::setprecision(4) << goal_gps_point.bearing
		   << ", x: " << std::setprecision(3) << goal_gps_point.x
		   << ", y: " << std::setprecision(3) << goal_gps_point.y
		   << ", cone: " << (goal_gps_point.has_cone ? "T" : "F");
		state_ = ROTATING_TO_HEADING;
		result = RUNNING;
		break;

	case ROTATING_TO_HEADING:
		// The robot may not be pionting towards the goal waypoint. If not, rotate so it is.
		if (fabs(goal_yaw_delta) < angles::from_degrees(goal_yaw_degrees_delta_threshold_)) {
			// Close enough to desired heading. Just move.
			ss << ". ROTATING_TO_HEADING and delta yaw (deg): " << std::setprecision(4) << angles::to_degrees(goal_yaw_delta)
			   << " is close enough. Changing state to SEEKING_POINT";
			state_ = SEEKING_POINT;
			result = RUNNING;
		} else {
			// Need to rotate towards target.
			// Just rotate a bit each time the behavior is invoked.
			cmd_vel.linear.x = linear_move_meters_per_sec_ / 2.0;
			cmd_vel.angular.z = goal_yaw_delta > 0 ? yaw_turn_radians_per_sec_ : -yaw_turn_radians_per_sec_;
			cmd_vel_pub_.publish(cmd_vel);

			ss << ". rot, x: " << std::setprecision(3) << cmd_vel.linear.x << ", z: " << std::setprecision(3) << cmd_vel.angular.z;
			result = RUNNING;
		}

		break;

	case SEEKING_POINT:
		// The robot is heading in the right direction and isn't close enough yet.
		// Move the robot forward a bit each time the behavior is invoked. 
		if (fabs(goal_yaw_delta) >= angles::from_degrees(goal_yaw_degrees_delta_threshold_)) {
			ss << ". SEEKING_POINT and delta_yaw (deg): "
			   << angles::to_degrees(goal_yaw_delta)
			   << " has become large. Changing state to ROTATING_TO_HEADING";
			state_ = ROTATING_TO_HEADING;
			result = RUNNING;
		} else {
			cmd_vel.linear.x = linear_move_meters_per_sec_;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << ". SEEKING_POINT, gofwd, x: " << std::setprecision(3) << cmd_vel.linear.x << ", z: " << std::setprecision(3) << cmd_vel.angular.z;
			result = RUNNING;
		}

		break;

	default:
		ss << ". INVALID STATE";
		break;
	}


	publishStrategyProgress("SeekToGps::tick", ss.str());
	return result;
}

SeekToGps& SeekToGps::singleton() {
    static SeekToGps singleton_;
    return singleton_;
}
