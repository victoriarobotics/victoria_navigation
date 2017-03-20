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
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/discover_cone.h"
#include "victoria_perception/ObjectDetector.h"

DiscoverCone::DiscoverCone() :
	count_ObjectDetector_msgs_received_(0),
	odometry_capturered_(false),
	state_(kCAPTURE_ODOMETRY)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~do_debug_strategy", do_debug_strategy_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);
	current_strategy_pub_ = nh_.advertise<std_msgs::String>("current_strategy", 1, true /* latched */);
	strategy_status_publisher_ = nh_.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &DiscoverCone::coneDetectorCb, this);
	odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &DiscoverCone::odometryCb, this);

	ROS_INFO("[DiscoverCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[DiscoverCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_INFO("[DiscoverCone] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
}

double DiscoverCone::normalizeEuler(double yaw) {
	double result = yaw;
	while (result > 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

void DiscoverCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_ObjectDetector_msg_ = *msg;
	count_ObjectDetector_msgs_received_++;
}

void DiscoverCone::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_Odometry_msg_ = *msg;
	count_Odometry_msgs_received_++;	
}

void DiscoverCone::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != last_reported_strategy_) {
		last_reported_strategy_ = strategy;
		current_strategy_pub_.publish(msg);
		ROS_INFO_COND(do_debug_strategy_, "[DiscoverCone] strategy: %s", strategy.c_str());
	}
}

void DiscoverCone::resetGoal() {
	odometry_capturered_ = false;
	state_ = kCAPTURE_ODOMETRY;
	ros::param::set("/strategy/need_to_discover_cone", false);
}

StrategyFn::RESULT_T DiscoverCone::tick() {
	geometry_msgs::Twist		cmd_vel;
	actionlib_msgs::GoalStatus 	goal_status;
	RESULT_T 					result = FATAL;
	ostringstream 				ss;

	bool need_to_discover_cone;
	if (!ros::param::get(goalRequestParam(), need_to_discover_cone) || (!need_to_discover_cone)) {
		ROS_INFO_COND(do_debug_strategy_, "[DiscoverCone::tick] FAILED: need_to_discover_cone not active");
		return FAILED;
	}

	if (count_ObjectDetector_msgs_received_ <= 0) {
		ROS_INFO_COND(do_debug_strategy_, "[DiscoverCone::tick] FAILED: no ConeDetector messages received");
		return FAILED; // No data yet.
	}

	if (last_ObjectDetector_msg_.object_detected) {
		ROS_INFO_COND(do_debug_strategy_, "[DiscoverCone::tick] SUCCESS: object detected");
		ros::param::set("/strategy/need_to_discover_cone", false);
		odometry_capturered_ = false;
		state_ = kCAPTURE_ODOMETRY;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub_.publish(cmd_vel);
		return SUCCESS; // Cone found.
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

	goal_status.goal_id.stamp = ros::Time::now();
	goal_status.goal_id.id = "DiscoverCone";
	goal_status.status = actionlib_msgs::GoalStatus::ACTIVE;

	if (state_ == kCAPTURE_ODOMETRY) {
		// Looking for an Odometry message so we can capture the heading at the start of the search.
		if (odometry_capturered_) {
			// Shouldn't get to this point. Still, just go on to the begin-rotation strategy.
			state_ = kROTATING_TO_DISCOVER;
			ss << "WARN strange state--in kCAPTURE_ODOMETRY but odometry_capturered_ already true";
		} else if (count_Odometry_msgs_received_ > 0) {
			starting_Odometry_msg_ = last_Odometry_msg_;
			previous_pose_ = last_Odometry_msg_.pose.pose.orientation;
			starting_yaw_ = normalizeEuler(tf::getYaw(starting_Odometry_msg_.pose.pose.orientation));
			total_rotated_yaw_ = 0;
			odometry_capturered_ = true;
			state_ = kROTATING_TO_DISCOVER;	// We have a starting orientation, go to the begin-rotation strategy.
			ss << "got Odometry, begin rotation";
		} else {
			// Waiting for Odometry messages to show up.
			ss << "waiting for Odometry messages to be received";
		}

		result = RUNNING;
	} else if (state_ == kROTATING_TO_DISCOVER) {
		// No cone yet discovered (it would have been handled above), so rotate a bit if not already
		// completed a full circle.
		tf::Quaternion previous_tf_quat;
		tf::Quaternion current_tf_quat;
		tf::quaternionMsgToTF(previous_pose_, previous_tf_quat);
		tf::quaternionMsgToTF(last_Odometry_msg_.pose.pose.orientation, current_tf_quat);
		double originalYaw = normalizeEuler(tf::getYaw(starting_Odometry_msg_.pose.pose.orientation) * 360.0 / (2 * M_PI));
		double currentYaw = normalizeEuler(tf::getYaw(last_Odometry_msg_.pose.pose.orientation) * 360.0 / (2 * M_PI));
		total_rotated_yaw_ += abs(previous_tf_quat.angleShortestPath(current_tf_quat));
		previous_pose_ = last_Odometry_msg_.pose.pose.orientation;

		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0.4; //TODO ### Make configurable.

		ss << "Rotating";
		ss << ", originalYaw: " << std::setprecision(5) << originalYaw;
		ss << ", currentYaw: " << std::setprecision(5) << currentYaw;
		ss << ", total_rotated_yaw_: " << total_rotated_yaw_;

		if (total_rotated_yaw_ > (2 * M_PI)) {
			ss << ", FAIL no cone found after one rotation";
			result = FAILED;
			resetGoal();
		} else {
			// Keep rotating.
			cmd_vel_pub_.publish(cmd_vel);
			result = RUNNING;
		}
	}

	goal_status.text = ss.str();
	strategy_status_publisher_.publish(goal_status);
	ROS_INFO_COND(do_debug_strategy_, "[DiscoverCone::tick] %s", ss.str().c_str());

	return result;
}

DiscoverCone& DiscoverCone::singleton() {
    static DiscoverCone singleton_;
    return singleton_;
}
