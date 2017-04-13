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
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/move_from_cone.h"

MoveFromCone::MoveFromCone() :
	state_(MOVE_START)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	ROS_INFO("[MoveFromCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
}

// Reset global state so this behavior can be used to solve the next problem.
void MoveFromCone::resetGoal() {
	state_ = MOVE_START;
}

StrategyFn::RESULT_T MoveFromCone::tick() {
	geometry_msgs::Twist		cmd_vel;		// For sending movement commands to the robot.
	RESULT_T 					result = FATAL;	// Assume fatality in the algorithm.
	std::ostringstream 			ss;				// For sending informations messages.
	ros::Duration move_duration = ros::Time::now() - move_start_time_;

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	switch (state_) {
	case MOVE_START:
		move_start_time_ = ros::Time::now();
		state_ = MOVING_AWAY;
		result = RUNNING;
		break;

	case MOVING_AWAY:
		if (move_duration.toSec() < 1.0) {
			cmd_vel.linear.x = -0.3;
			cmd_vel.angular.z = 0.0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "Backing up, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
		} else {
			resetGoal();
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "SUCCESS backed up.";
			publishStrategyProgress("MoveFromCone::tick", ss.str());
			popGoal();
			return setGoalResult(SUCCESS);
		}

		result = RUNNING;
		break;

	default:
		ss << "FATAL: invalid state";
		return setGoalResult(FATAL);
	}

	publishStrategyProgress("MoveFromCone::tick", ss.str());
	return result;
}

StrategyFn& MoveFromCone::singleton() {
    static MoveFromCone singleton_;
    return singleton_;
}
