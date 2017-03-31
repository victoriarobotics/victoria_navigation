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
	state_(kSETUP)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~fix_topic_name", fix_topic_name_));
	assert(ros::param::get("~gps_close_distance_meters", gps_close_distance_meters_));
	assert(ros::param::get("~imu_topic_name", imu_topic_name_));
	assert(ros::param::get("~magnetic_declination", magnetic_declination_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	assert(ros::param::get("~solve_using_odom", solve_using_odom_));
	assert(ros::param::get("~use_imu", use_imu_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	assert(cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &SeekToGps::coneDetectorCb, this));
	assert(fix_sub_ = nh_.subscribe(fix_topic_name_, 1, &SeekToGps::fixCb, this));
	assert(imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &SeekToGps::imuCb, this));

	assert(odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &SeekToGps::odometryCb, this));

	ROS_INFO("[SeekToGps] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM fix_topic_name_: %s", fix_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM gps_close_distance_meters: %7.4f", gps_close_distance_meters_);
	ROS_INFO("[SeekToGps] PARAM imu_topic_name_: %s", imu_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM magnetic_declination_: %7.4f", magnetic_declination_);
	ROS_INFO("[SeekToGps] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_INFO("[DiscoverCone] PARAM solve_using_odom: %s", solve_using_odom_ ? "TRUE" : "FALSE");
	ROS_INFO("[SeekToGps] PARAM use_imu: %s", use_imu_ ? "TRUE" : "FALSE");
}

// Convert an Euler angle into a range of [0 .. 360).
double SeekToGps::normalizeEuler(double yaw) {
	double result = yaw;
	while (result >= 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

// Capture the lates ConeDetector information
void SeekToGps::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_ObjectDetector_msg_ = *msg;
	count_ObjectDetector_msgs_received_++;
}

// Capture the lates Fix information.
void SeekToGps::fixCb(const sensor_msgs::NavSatFixConstPtr& msg) {
	last_Fix_msg_ = *msg;
	count_Fix_msgs_received_++;
}

// Capture the latest Imu information.
void SeekToGps::imuCb(const sensor_msgs::ImuConstPtr& msg) {
	last_Imu_msg_ = *msg;
	count_Imu_msgs_received_++;
}

// Capture the lates Odometry information.
void SeekToGps::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_Odometry_msg_ = *msg;
	count_Odometry_msgs_received_++;	
}

// Reset global state so this behavior can be used to solve the next problem.
void SeekToGps::resetGoal() {
	state_ = kSEEKING_POINT;
}

double SeekToGps::odomBearing(double x1, double y1, double x2, double y2) {
    if ((x1 == x2) && (y1 == y2)) return 0;
    double theta = atan2(x2 - x1, y1 - y2);
    if (theta < 0.0)
        theta += 2 * M_PI;
    return radians(theta);
}

StrategyFn::RESULT_T SeekToGps::tick() {
	geometry_msgs::Twist		cmd_vel;		// For sending movement commands to the robot.
	RESULT_T 					result = FATAL;	// Assume fatality in the algorithm.
	ostringstream 				ss;				// For sending informations messages.

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_ObjectDetector_msgs_received_ <= 0) {
		// Wait until ConeDetector messages are received.
		return RUNNING;
	}

	if (count_Odometry_msgs_received_ <= 0) {
		// Wait until Odometry messages are received.
		return RUNNING;
	}

	if (count_Fix_msgs_received_ <= 0) {
		// Wait until Fix messages are received.
		return RUNNING;
	}

	if (count_Imu_msgs_received_ <= 0) {
		// Wait until Imu messages are received.
		return RUNNING;
	}

	const GPS_POINT& gps_point = currentGpsPoint();	// Get the goal waypoint.

	if (gps_point.has_cone && last_ObjectDetector_msg_.object_detected) {
		// Success! Stop when a RoboMagellan cone is seen.
		resetGoal();
		
		cmd_vel.linear.x = 0;	// Stop motion.
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);

		// Publish information.
		ss << "SUCCESS Object detected, STOP";
		ss << ", area: " << last_ObjectDetector_msg_.object_area;
		publishStrategyProgress("SeekToGps::tick", ss.str());
		
		// Standard way to indicate success.
		popGoal();
		return setGoalResult(SUCCESS);
	}

	// Convert the goal waypoint into a NavSatFix format.
	sensor_msgs::NavSatFix point_as_fix;
	point_as_fix.latitude = gps_point.latitude;
	point_as_fix.longitude = gps_point.longitude;

	// Compute the heading and distance from the current position to the goal waypoint.
	double point_bearing_gps_degrees;
	if (solve_using_odom_) {
		point_bearing_gps_degrees = odomBearing(last_Odometry_msg_.pose.pose.position.x, last_Odometry_msg_.pose.pose.position.y,
												gps_point.x, gps_point.y);
	} else {
		point_bearing_gps_degrees = bearing(last_Fix_msg_, point_as_fix);
	}

	double point_distance;
	if (solve_using_odom_) {
		point_distance = sqrt(((last_Odometry_msg_.pose.pose.position.x - gps_point.x) * (last_Odometry_msg_.pose.pose.position.x - gps_point.x)) +
							  ((last_Odometry_msg_.pose.pose.position.y - gps_point.y) * (last_Odometry_msg_.pose.pose.position.y - gps_point.y)));
	} else {
		point_distance = distance(last_Fix_msg_, point_as_fix);
	}

	// Begin forming an informational message.
	ss << std::ios::fixed
	   << "AT lat: " << std::setprecision(9) << last_Fix_msg_.latitude
	   << ", lon: " << std::setprecision(9) << last_Fix_msg_.longitude
	   << ", x: " << std::setprecision(3) << last_Odometry_msg_.pose.pose.position.x
	   << ", y: " << std::setprecision(3) << last_Odometry_msg_.pose.pose.position.y
	   << ", TO lat: " << std::setprecision(9) << gps_point.latitude
	   << ", lon: " << std::setprecision(9) << gps_point.longitude
	   << ", head: " << std::setprecision(4) << gps_point.bearing
	   << ", x: " << std::setprecision(3) << gps_point.x
	   << ", y: " << std::setprecision(3) << gps_point.y
	   << ", cone: " << (gps_point.has_cone ? "T" : "F")
	   << ", distance: "  << std::setprecision(3) << point_distance;

	// Is the robot "close enough"?
	if (point_distance < gps_close_distance_meters_) {
		// Close enough.
		resetGoal();	// Reset the global state so the behavior can be used for another problem.

		// Stop the robot.
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);

		// Publish information.
		ss << ". SUCCESS close to point, STOP";
		publishStrategyProgress("SeekToGps::tick", ss.str());

		// Standard way to indicate success.
		popGoal();
		return setGoalResult(SUCCESS);
	}

	// Compute the direction the robot needs to be pointing to.
	goal_yaw_degrees_ = normalizeEuler(90 - point_bearing_gps_degrees);; // Convert GPS heading to ROS heading.
	double goal_yaw_degrees_delta = 0; // How far off is the current heading from the desired heading?
	double corrected_degrees = 0;  // Current heading corrected, if necessary, by magnetic declination.

	if (use_imu_) {
		// Use the Imu as truth for the current robot heading.
		tf::Quaternion imu_orientation;
		tf::quaternionMsgToTF(last_Imu_msg_.orientation, imu_orientation);
		double imu_yaw_degrees = normalizeEuler(tf::getYaw(imu_orientation) * 360.0 / (2 * M_PI));
		corrected_degrees = imu_yaw_degrees + magnetic_declination_;
		ss << ", corrected_yaw_deg: " << std::setprecision(4) << corrected_degrees;
	} else {
		// Use the Odometry as truth for the current robot heading.
		tf::Quaternion odom_orientation;
		tf::quaternionMsgToTF(last_Odometry_msg_.pose.pose.orientation, odom_orientation);
		double odometry_yaw_degrees = normalizeEuler(tf::getYaw(odom_orientation) * 360.0 / (2 * M_PI));
		corrected_degrees = odometry_yaw_degrees;
		ss << ", odometry_yaw_deg: " << std::setprecision(4) << odometry_yaw_degrees;
	}

	goal_yaw_degrees_delta = goal_yaw_degrees_ - corrected_degrees;  // How far off is the current heading?
	ss << ", goal_deg: " << std::setprecision(4) << goal_yaw_degrees_;
	ss << ", goal_degrees_d: " << std::setprecision(4) << goal_yaw_degrees_delta;
	if (state_ == kSETUP) {
		// The first time through, just gather data.
		// Set up so that the next time through, the robot is rotated to point to the goal waypoint.
		ss << ". Setting up";
		state_ = kROTATING_TO_HEADING;
		result = RUNNING;
	} else if (state_ == kROTATING_TO_HEADING) {
		// The robot may not be pionting towards the goal waypoint. If not, rotate so it is.
		if (abs(goal_yaw_degrees_delta) < goal_yaw_degrees_delta_threshold_) {
			// Close enough to desired heading. Just move.
			ss << ". In state kROTATING_TO_HEADING and delta yaw: " << std::setprecision(4) << goal_yaw_degrees_delta
			   << " is close enough. Changing state to kSEEKING_POINT";
			state_ = kSEEKING_POINT;
			result = RUNNING;
		} else {
			// Need to rotate towards target.
			// Just rotate a bit each time the behavior is invoked.
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = goal_yaw_degrees_delta > 0 ? 0.4 : -0.4; //TODO ### Make configurable.
			cmd_vel_pub_.publish(cmd_vel);

			ss << ". Rotating, z: " << std::setprecision(3) << cmd_vel.angular.z;
			result = RUNNING;
		}
	} else if (state_ == kSEEKING_POINT) {
		// The robot is heading in the right direction and isn't close enough yet.
		// Move the robot forward a bit each time the behavior is invoked. 
		if (abs(goal_yaw_degrees_delta) >= goal_yaw_degrees_delta_threshold_) {
			ss << ". In state kSEEKING_POINT and delta_yaw: "
			   << goal_yaw_degrees_delta
			   << " has become large. Changing state to kROTATING_TO_HEADING";
			state_ = kROTATING_TO_HEADING;
			result = RUNNING;
		} else {
			cmd_vel.linear.x = 0.2;  // TODO ### Make configurable.
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << ". In state kSEEKING_POINT, moving forward, x: " << std::setprecision(3) << cmd_vel.linear.x;
			result = RUNNING;
		}
	} else {
		ss << ". INVALID STATE";
	}


	publishStrategyProgress("SeekToGps::tick", ss.str());
	return result;
}

SeekToGps& SeekToGps::singleton() {
    static SeekToGps singleton_;
    return singleton_;
}
