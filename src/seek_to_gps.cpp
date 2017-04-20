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
	heading_to_waypoint_degrees_delta_threshold_(3.0),
	linear_move_meters_per_sec_(0.4),
	state_(SETUP),
	yaw_turn_radians_per_sec_(0.4)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~fix_topic_name", fix_topic_name_));
	assert(ros::param::get("~gps_close_distance_meters", gps_close_distance_meters_));
	assert(ros::param::get("~ignore_cone_until_within_meters", ignore_cone_until_within_meters_));
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
	ROS_INFO("[SeekToGps] PARAM ignore_cone_until_within_meters: %7.4f", ignore_cone_until_within_meters_);
	ROS_INFO("[SeekToGps] PARAM imu_topic_name: %s", imu_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM linear_move_meters_per_sec: %7.4f", linear_move_meters_per_sec_);
	ROS_INFO("[SeekToGps] PARAM magnetic_declination: %7.4f", magnetic_declination_);
	ROS_INFO("[SeekToGps] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_INFO("[SeekToGps] PARAM solve_using_odom: %s", solve_using_odom_ ? "TRUE" : "FALSE");
	ROS_INFO("[SeekToGps] PARAM use_imu: %s", use_imu_ ? "TRUE" : "FALSE");
	ROS_INFO("[SeekToGps] PARAM yaw_turn_radians_per_sec: %7.4f", yaw_turn_radians_per_sec_);

    coneDetectorAnnotatorService_ = nh_.serviceClient<victoria_perception::AnnotateDetectorImage>("/cone_detector/annotate_detector_image");
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

double SeekToGps::odomHeading(double x1, double y1, double x2, double y2) {
    if ((x1 == x2) && (y1 == y2)) return 0;
    double theta = -atan2(y1 - y2, x2 - x1 );
    theta = angles::normalize_angle(theta);

    return theta;
}

StrategyFn::RESULT_T SeekToGps::tick() {
	bool						force_report = false; 	// Force status report.
	geometry_msgs::Twist		cmd_vel;				// For sending movement commands to the robot.
	RESULT_T 					result = FATAL;			// Assume fatality in the algorithm.
	std::ostringstream 				ss;					// For sending informations messages.

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_object_detector_msgs_received_ <= 0) {
		// Wait until ConeDetector messages are received.
		ROS_INFO("[SeekToGps::tick] waiting ObjectDetector messages");
	    annotator_request_.request.annotation = "UL;FFFFFF;STG Cone wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	if (count_odometry_msgs_received_ <= 0) {
		// Wait until Odometry messages are received.
		ROS_INFO("[SeekToGps::tick] waiting Odometry messages");
	    annotator_request_.request.annotation = "UL;FFFFFF;STG Odom wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	if (count_fix_msgs_received_ <= 0) {
		// Wait until Fix messages are received.
		ROS_INFO("[SeekToGps::tick] waiting Fix messages");
	    annotator_request_.request.annotation = "UL;FFFFFF;STG Fix wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	if (count_imu_msgs_received_ <= 0) {
		// Wait until Imu messages are received.
		ROS_INFO("[SeekToGps::tick] waiting Imu messages");
	    annotator_request_.request.annotation = "UL;FFFFFF;STG Imu wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	const GPS_POINT& waypoint = currentGpsPoint();	// Get the goal waypoint.
	ss << "pt: " << waypoint.point_number << " ";

	// Convert the goal waypoint into a NavSatFix format.
	sensor_msgs::NavSatFix waypoint_as_nav_sat_fix;
	waypoint_as_nav_sat_fix.latitude = waypoint.latitude;
	waypoint_as_nav_sat_fix.longitude = waypoint.longitude;

	// Compute the heading and distance from the current position to the goal waypoint.
	double robot_to_waypoint_heading;
	if (solve_using_odom_) {
		robot_to_waypoint_heading = odomHeading(last_odometry_msg_.pose.pose.position.x, last_odometry_msg_.pose.pose.position.y,
												waypoint.x, waypoint.y);
	} else {
		// GPS calculatio results in GPS degreess, which is 90 degrees off from the ROS framework.
		robot_to_waypoint_heading = angles::normalize_angle((M_PI / 2.0) - headingInGpsCoordinates(last_fix_msg_, waypoint_as_nav_sat_fix));
	}

	double robot_to_waypoint_distance;
	if (solve_using_odom_) {
		robot_to_waypoint_distance = sqrt(((last_odometry_msg_.pose.pose.position.x - waypoint.x) * (last_odometry_msg_.pose.pose.position.x - waypoint.x)) +
							  ((last_odometry_msg_.pose.pose.position.y - waypoint.y) * (last_odometry_msg_.pose.pose.position.y - waypoint.y)));
	} else {
		robot_to_waypoint_distance = greatCircleDistance(last_fix_msg_, waypoint_as_nav_sat_fix);
	}

	// Compute the heading the robot needs to be pointing.
	double heading_to_waypoint_delta = 0; // How far off is the current heading from the desired heading?
	double robot_true_heading = 0;  // Current heading corrected, if necessary, by magnetic declination.

	if (use_imu_) {
		// Use the Imu as truth for the current robot heading.
		tf::Quaternion imu_orientation;
		tf::quaternionMsgToTF(last_imu_msg_.orientation, imu_orientation);
		double imu_yaw = tf::getYaw(imu_orientation);
		robot_true_heading = imu_yaw + angles::from_degrees(magnetic_declination_);
	} else {
		// Use the Odometry as truth for the current robot heading.
		tf::Quaternion odom_orientation;
		tf::quaternionMsgToTF(last_odometry_msg_.pose.pose.orientation, odom_orientation);
		robot_true_heading = tf::getYaw(odom_orientation);
	}

	heading_to_waypoint_delta = angles::shortest_angular_distance(robot_true_heading, robot_to_waypoint_heading);  // How far off is the current heading?

	if (waypoint.has_cone &&
		last_object_detector_msg_.object_detected &&
		(robot_to_waypoint_distance <= ignore_cone_until_within_meters_)) {
		// Success! Stop when a RoboMagellan cone is seen.
		resetGoal();
		
		cmd_vel.linear.x = 0;	// Stop motion.
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);

		// Publish information.
		ss << "SUCCESS Object detected, STOP";
		ss << ", cone area: " << last_object_detector_msg_.object_area;
		publishStrategyProgress("SeekToGps::tick", ss.str(), true);
		
	    annotator_request_.request.annotation = "UL;FFFFFF;STG cone SUCCESS";
	    coneDetectorAnnotatorService_.call(annotator_request_);

		// Standard way to indicate success.
		popGoal();
		return setGoalResult(SUCCESS);
	}

	// Add to the status message.
	char msg[256];
	sprintf(msg, "%s head: % #6.1f, goal head: % #6.1f, dist: % #5.1f",
			use_imu_ ? "IMU" : "ODOM",
			angles::to_degrees(robot_true_heading),
			angles::to_degrees(robot_to_waypoint_heading),
			robot_to_waypoint_distance);
	ss << msg;

	// Is the robot "close enough"?
	if (robot_to_waypoint_distance < gps_close_distance_meters_) {
		// Close enough.
		resetGoal();	// Reset the global state so the behavior can be used for another problem.

		// Publish information.
		ss << ". SUCCESS close to goal, STOP";
		publishStrategyProgress("SeekToGps::tick", ss.str(), true);

	    annotator_request_.request.annotation = "UL;FFFFFF;STG close SUCCESS";
	    coneDetectorAnnotatorService_.call(annotator_request_);

		// Standard way to indicate success.
		popGoal();
		return setGoalResult(SUCCESS);
	}

	// Update the dashboard annotation.
	sprintf(msg, "UL;FFFFFF;STG pt %d head: %3.1f dist %3.1f",
			waypoint.point_number,
			angles::to_degrees(robot_to_waypoint_heading),
			robot_to_waypoint_distance);
    annotator_request_.request.annotation = msg;
    coneDetectorAnnotatorService_.call(annotator_request_);


	switch (state_) {
	case SETUP:
		// The first time through, just gather data.
		// Set up so that the next time through, the robot is rotated to point to the goal waypoint.
		ss << ". Setting up";
		if (solve_using_odom_) {
			ss << ", TO x: " << std::setprecision(3) << waypoint.x
		       << ", y: " << std::setprecision(3) << waypoint.y;
	    } else {
		   ss << ", TO lat: " << std::setprecision(11) << waypoint.latitude
		      << ", lon: " << std::setprecision(11) << waypoint.longitude
		      << ", GPS head: " << std::setprecision(4) << waypoint.bearing;
        }

        ss << ", distance: " << std::setprecision(4) << robot_to_waypoint_distance;
        ss << ", heading: " << std::setprecision(4) << robot_to_waypoint_heading;
		ss << ", has_cone: " << (waypoint.has_cone ? "T" : "F");
		state_ = ROTATING_TO_HEADING;
		result = RUNNING;
		force_report = true;	// Make sure this status gets reported.
		break;

	case ROTATING_TO_HEADING:
		// The robot may not be pointing towards the goal waypoint. If not, rotate it a bit so it is closer to the right heading.
		if (fabs(heading_to_waypoint_delta) < angles::from_degrees(heading_to_waypoint_degrees_delta_threshold_)) {
			// Close enough to desired heading. Just move.
			state_ = SEEKING_POINT;
			result = RUNNING;
		} else {
			// Need to rotate towards the goal heading.
			// Just rotate a bit each time the behavior is invoked.
			cmd_vel.linear.x = linear_move_meters_per_sec_ / 2.0;
			cmd_vel.angular.z = heading_to_waypoint_delta > 0 ? yaw_turn_radians_per_sec_ : -yaw_turn_radians_per_sec_;
			cmd_vel_pub_.publish(cmd_vel);
			result = RUNNING;
		}

		break;

	case SEEKING_POINT:
		// The robot is heading in the right direction and isn't close enough yet.
		// Move the robot forward a bit each time the behavior is invoked. 
		if (fabs(heading_to_waypoint_delta) >= angles::from_degrees(heading_to_waypoint_degrees_delta_threshold_)) {
			// Now the robot heading is too far from the desired heading.
			// Change the state so the state machine will try to correct the heading.
			state_ = ROTATING_TO_HEADING;
			result = RUNNING;
		} else {
			// Move forward in a straight line.
			cmd_vel.linear.x = linear_move_meters_per_sec_;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			result = RUNNING;
		}

		break;

	default:
		ss << ". INVALID STATE";
		break;
	}

	publishStrategyProgress("SeekToGps::tick", ss.str(), force_report);
	return result;
}

StrategyFn& SeekToGps::singleton() {
    static SeekToGps singleton_;
    return singleton_;
}