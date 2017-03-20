// Copyright <YEAR> <COPYRIGHT HOLDER>

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
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/move_to_cone.h"
#include "victoria_perception/ObjectDetector.h"

MoveToCone::MoveToCone() :
	object_detections_received_(0),
	state_(kMOVING_TO_CENTERING_POSITION)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~do_debug_strategy", do_debug_strategy_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);
	current_strategy_pub_ = nh_.advertise<std_msgs::String>("current_strategy", 1, true /* latched */);
	strategy_status_publisher_ = nh_.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &MoveToCone::coneDetectorCb, this);

	ROS_INFO("[MoveToCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[MoveToCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
}

void MoveToCone::resetGoal() {
	state_ = kMOVING_TO_CENTERING_POSITION;
	ros::param::set("/strategy/need_to_move_cone", false);
}

void MoveToCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detected_ = *msg;
	object_detections_received_++;
}

void MoveToCone::publishCurrentStragety(string strategy) {
	std_msgs::String msg;
	msg.data = strategy;
	if (strategy != last_reported_strategy_) {
		last_reported_strategy_ = strategy;
		current_strategy_pub_.publish(msg);
		ROS_INFO_COND(do_debug_strategy_, "[MoveToCone] strategy: %s", strategy.c_str());
	}
}

StrategyFn::RESULT_T MoveToCone::tick() {
	geometry_msgs::Twist		cmd_vel;
	actionlib_msgs::GoalStatus 	goal_status;
	RESULT_T 					result = FATAL;
	ostringstream 				ss;

	bool need_to_move_to_cone;
	if (!ros::param::get(goalRequestParam(), need_to_move_to_cone) || (!need_to_move_to_cone)) {
		ROS_INFO_COND(do_debug_strategy_, "[MoveToCone::tick] FAILED: need_to_move_to_cone not active");
		return FAILED;
	}

	if (object_detections_received_ <= 0) {
		ROS_INFO_COND(do_debug_strategy_, "[MoveToCone::tick] FAILED: no ConeDetector messages received");
		return FAILED; // No data yet.
	}

	if (!last_object_detected_.object_detected) {
		ROS_INFO_COND(do_debug_strategy_, "[MoveToCone::tick] FAILED: no object detected");
		return FAILED; // No cone found.
	}

	goal_status.goal_id.stamp = ros::Time::now();
	goal_status.goal_id.id = "MoveToCone";
	goal_status.status = actionlib_msgs::GoalStatus::ACTIVE;

	if (state_ == kMOVING_TO_CENTERING_POSITION) {
		long threshold_area = (last_object_detected_.image_height * last_object_detected_.image_width / 2);
		if (last_object_detected_.object_area > threshold_area) {
			//### Fake test to stop when close.
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "[MoveToCone::tick]";
			ss << " close, STOP";
			ss << ", area: " << last_object_detected_.object_area;
			ss << ", threshold_area: " << threshold_area;
			resetGoal();
			result = SUCCESS;
		} else {
			result = RUNNING;
			int image_width = last_object_detected_.image_width;
			int object_x = last_object_detected_.object_x;
			if (abs((image_width / 2) - object_x) < (image_width / 40)) {
				// Heading generally in the right direction.
				cmd_vel.linear.x = 0.15; //### Arbitrary.
				cmd_vel.angular.z = 0;
				cmd_vel_pub_.publish(cmd_vel);
				ss << "[MoveToCone::tick]";
				ss << " go straight, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
				ss << ", area: " << last_object_detected_.object_area;
			} else {
				// Turn towards cone.
				cmd_vel.linear.x = 0.2; //### Arbitrary.
				cmd_vel.angular.z = ((image_width / 2.0) - last_object_detected_.object_x) / last_object_detected_.image_width;
				cmd_vel_pub_.publish(cmd_vel);
				ss << "[MoveToCone::tick]";
				ss << " turn, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
				ss << ", area: " << last_object_detected_.object_area;
			}
		}
	} else if (state_ == kMOVING_TO_TOUCH) {
		ROS_ERROR("[MoveToCone::tick] FAILED: Unimplemented state kMOVING_TO_TOUCH");
		return FAILED;
	} else {
		ROS_ERROR("[MoveToCone::tick] FAILED: invalid state");
		return FAILED;
	}

	goal_status.text = ss.str();
	strategy_status_publisher_.publish(goal_status);
	ROS_INFO_COND(do_debug_strategy_, "[MoveToCone::tick] %s", ss.str().c_str());

	return result;
}

MoveToCone& MoveToCone::singleton() {
    static MoveToCone singleton_;
    return singleton_;
}
