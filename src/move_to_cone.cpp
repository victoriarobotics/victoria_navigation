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

#include "victoria_navigation/move_to_cone.h"
#include "victoria_perception/ObjectDetector.h"

MoveToCone::MoveToCone() :
	count_ObjectDetector_msgs_received_(0),
	sequential_detection_failures_(0),
	state_(MOVE_START)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &MoveToCone::coneDetectorCb, this);

	ROS_INFO("[MoveToCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[MoveToCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
}

// Capture the lates ConeDetector information
void MoveToCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detected_ = *msg;
	count_ObjectDetector_msgs_received_++;
}

// Reset global state so this behavior can be used to solve the next problem.
void MoveToCone::resetGoal() {
	state_ = MOVE_START;
}

StrategyFn::RESULT_T MoveToCone::tick() {
	geometry_msgs::Twist		cmd_vel;		// For sending movement commands to the robot.
	RESULT_T 					result = FATAL;	// Assume fatality in the algorithm.
	std::ostringstream 				ss;				// For sending informations messages.

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_ObjectDetector_msgs_received_ <= 20) {
		// Wait until ConeDetector messages are received.
		return RUNNING;
	}

	if (state_ == MOVE_START) {
		time_last_saw_cone = ros::Time::now();
		state_ = MOVING_TO_CENTERING_POSITION;
		return RUNNING;
	}
// FAILED, lost object detection. time_last_saw_cone: 0.000000000, now: 1491949414.455211667, duration_since_last_saw_cone: 1491949414.455211667

	if (!last_object_detected_.object_detected) {
		// Failure, lost sight of the cone.
		ros::Time now = ros::Time::now();
		ros::Duration duration_since_last_saw_cone = now - time_last_saw_cone;
		if (duration_since_last_saw_cone.toSec() > 5) {
			resetGoal();

			cmd_vel.linear.x = 0;	// Stop motion.
			cmd_vel.angular.z = 0.0;
			cmd_vel_pub_.publish(cmd_vel);

			// Publish information.
			ss << "FAILED, lost object detection. time_last_saw_cone: " << time_last_saw_cone;
			ss << ", now: " << now;
			ss << ", duration_since_last_saw_cone: " << duration_since_last_saw_cone;
			publishStrategyProgress("MoveToCone::tick", ss.str());

			// Standard way to indicate failure.
			popGoal();
			return setGoalResult(FAILED); // No object found.
		} else {
			// Just ignore object detection failure for a while to see if it's a fluke.
			// Note we don't stop motor here. We rely on the motor controller to time out.
			return RUNNING;
		}
	} else {
		sequential_detection_failures_ = 0;
		time_last_saw_cone = ros::Time::now();
	}

	int image_width = 0;
	int object_x = 0;
	long threshold_area = 0;

	switch (state_) {
	case MOVE_START:
		time_last_saw_cone = ros::Time::now();
		state_ = MOVING_TO_CENTERING_POSITION;
		result = RUNNING;
		break;

	case MOVING_TO_CENTERING_POSITION:
		threshold_area = (last_object_detected_.image_height * last_object_detected_.image_width / 2);
		if (last_object_detected_.object_area > threshold_area) {
			//### Fake test to stop when close.
			resetGoal();
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << " SUCCESS Close to object, STOP";
			ss << ", area: " << last_object_detected_.object_area;
			ss << ", threshold_area: " << threshold_area;
			publishStrategyProgress("MoveToCone::tick", ss.str());
			popGoal();
			return setGoalResult(SUCCESS);
		} 
	
		image_width = last_object_detected_.image_width;
		object_x = last_object_detected_.object_x;
		if (abs((image_width / 2) - object_x) < (image_width / 40)) {
			// Heading generally in the right direction.
			cmd_vel.linear.x = 0.15; //### Arbitrary.
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "Go straight, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
			ss << ", area: " << last_object_detected_.object_area;
		} else {
			// Turn towards cone.
			if (last_object_detected_.image_width > 0) {
				cmd_vel.linear.x = 0.2; //### Arbitrary.
				cmd_vel.angular.z = ((image_width / 2.0) - last_object_detected_.object_x) / last_object_detected_.image_width;
				cmd_vel_pub_.publish(cmd_vel);
				ss << "Turn, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
				ss << ", area: " << last_object_detected_.object_area;
			} else {
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				cmd_vel_pub_.publish(cmd_vel);
				ss << "FAULT! image.width <= 0, stoppiong";
			}			
		}

		result = RUNNING;
		break;

	case MOVING_TO_TOUCH: //#####
		ss << "FATAL: Unimplemented state MOVING_TO_TOUCH";
		return setGoalResult(FATAL);

	default:
		ss << "FATAL: invalid state";
		return setGoalResult(FATAL);
	}

	publishStrategyProgress("MoveToCone::tick", ss.str());
	return result;
}

MoveToCone& MoveToCone::singleton() {
    static MoveToCone singleton_;
    return singleton_;
}
