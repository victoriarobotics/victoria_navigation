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

#include <angles/angles.h>
#include <cassert>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/discover_cone.h"
#include "victoria_perception/ObjectDetector.h"

DiscoverCone::DiscoverCone() :
	count_object_detector_msgs_received_(0),
	state_(CAPTURE_ODOMETRY)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	assert(ros::param::get("~yaw_turn_radians_per_sec", yaw_turn_radians_per_sec_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &DiscoverCone::coneDetectorCb, this);
	odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &DiscoverCone::odometryCb, this);

	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone", "[DiscoverCone] PARAM yaw_turn_radians_per_sec: %7.4f", yaw_turn_radians_per_sec_);

    coneDetectorAnnotatorService_ = nh_.serviceClient<victoria_perception::AnnotateDetectorImage>("/cone_detector/annotate_detector_image", true);
}

// Capture the lates ConeDetector information
void DiscoverCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detector_msg_ = *msg;
	count_object_detector_msgs_received_++;
}

// Capture the lates Odometry information.
void DiscoverCone::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_odometry_msg_ = *msg;
	count_odometry_msgs_received_++;	
}

// Reset global state so this behavior can be used to solve the next problem.
void DiscoverCone::resetGoal() {
	state_ = CAPTURE_ODOMETRY;
}

StrategyFn::RESULT_T DiscoverCone::tick() {
	geometry_msgs::Twist		cmd_vel;		// For sending movement commands to the robot.
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
		// Success! Stop when a RoboMagellan cone is seen.
		resetGoal();

		cmd_vel.linear.x = 0;	// Stop motion.
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);

		// Publish information.
		ss << " SUCCESS Object detected, STOP";
		ss << ", area: " << last_object_detector_msg_.object_area;
		publishStrategyProgress("DiscoverCone::tick", ss.str());

		// Standard way to indicate success.
		popGoal();
	    annotator_request_.request.annotation = "UL;FFFFFF;DC cone SUCCESS";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return setGoalResult(SUCCESS); // Cone found.
	}

	// We have begun to receive ConeDetector messages but have not yet seen the cone.
	// Strategy: capture the current heading and begin a slow rotate to see if the
	// cone is detected in at most one revolution. If not, fail. If so, succeed.
	// TODO: This relies on receiving an Odometry message recently. No check is made
	// for freshness of the message--it's possible the Odometry information is from
	// a long time ago and no longer relevant. Also, it may be the robot isn't
	// publishing Odometry message. An alternate strategy would be to just begin rotation
	// and, knowing how fast the robot should be rotating, to stop when the robot should
	// have completed at least one 360 degree rotation.
	tf::Quaternion previous_tf_quat;
	tf::Quaternion current_tf_quat;
	double original_yaw = 0.0;
	double current_yaw = 0.0;

	switch (state_) {
	case CAPTURE_ODOMETRY:
		starting_odometry_msg_ = last_odometry_msg_;
		previous_pose_ = last_odometry_msg_.pose.pose.orientation;
		starting_yaw_ = tf::getYaw(starting_odometry_msg_.pose.pose.orientation);
		total_rotated_yaw_ = 0;
		state_ = ROTATING_TO_DISCOVER;	// We have a starting orientation, go to the begin-rotation strategy.
		ss << "Got Odometry, begin rotation";
		result = RUNNING;
		break;

	case ROTATING_TO_DISCOVER:
		// No cone yet discovered (it would have been handled above), so rotate a bit if not already
		// completed a full circle.
		tf::quaternionMsgToTF(previous_pose_, previous_tf_quat);
		tf::quaternionMsgToTF(last_odometry_msg_.pose.pose.orientation, current_tf_quat);
		original_yaw = tf::getYaw(starting_odometry_msg_.pose.pose.orientation);
		current_yaw = tf::getYaw(last_odometry_msg_.pose.pose.orientation);
		total_rotated_yaw_ = total_rotated_yaw_ + fabs(previous_tf_quat.angleShortestPath(current_tf_quat));
		previous_pose_ = last_odometry_msg_.pose.pose.orientation;

		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = yaw_turn_radians_per_sec_;

		ss << "Rotating";
		ss << ", original_yaw (deg): " << std::setprecision(5) << angles::to_degrees(original_yaw);
		ss << ", current_yaw (deg): " << std::setprecision(5) << angles::to_degrees(current_yaw);
		ss << ", total_rotated_yaw_ (deg): " << angles::to_degrees(total_rotated_yaw_);

		if (total_rotated_yaw_ > (2 * M_PI)) {
			ss << ", FAILED no cone found after one rotation";
			result = setGoalResult(FAILED);
			popGoal();
			resetGoal();
		    annotator_request_.request.annotation = "UL;FFFFFF;DC FAIL";
		    coneDetectorAnnotatorService_.call(annotator_request_);
		} else {
			// Keep rotating.
			cmd_vel_pub_.publish(cmd_vel);
			result = RUNNING;
		}

	    annotator_request_.request.annotation = "UL;FFFFFF;DC rotating";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		break;

	default:
		ss << "INVALID STATE";
		break;
	}

	publishStrategyProgress("DiscoverCone::tick", ss.str());
	return result;
}

StrategyFn& DiscoverCone::singleton() {
    static DiscoverCone singleton_;
    return singleton_;
}
