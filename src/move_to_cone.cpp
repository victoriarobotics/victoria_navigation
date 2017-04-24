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
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/move_to_cone.h"
#include "victoria_perception/ObjectDetector.h"

MoveToCone::MoveToCone() :
	bumper_hit_(false),
	count_ObjectDetector_msgs_received_(0),
	sequential_detection_failures_(0),
	state_(MOVE_START)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~cone_area_for_bumper_hit", cone_area_for_bumper_hit_));
	assert(ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_));
	assert(ros::param::get("~distance_displacement_1d_topic_name", distance_displacement_1d_topic_name_));
	assert(ros::param::get("~equate_size_to_bumper_hit", equate_size_to_bumper_hit_));
	assert(ros::param::get("~field_of_view_degrees", field_of_view_degrees_));
	assert(ros::param::get("~linear_move_meters_per_sec", linear_move_meters_per_sec_));
	assert(ros::param::get("~yaw_turn_radians_per_sec", yaw_turn_radians_per_sec_));
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &MoveToCone::coneDetectorCb, this);
	distance_displacement_1d_topic_name_sub_ = nh_.subscribe(distance_displacement_1d_topic_name_, 1, &MoveToCone::distanceDisplacement1DCb, this);

	ROS_INFO("[MoveToCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[MoveToCone] PARAM cone_area_for_bumper_hit: %d", cone_area_for_bumper_hit_);
	ROS_INFO("[MoveToCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
	ROS_INFO("[MoveToCone] PARAM distance_displacement_1d_topic_name_: %s", distance_displacement_1d_topic_name_.c_str());
	ROS_INFO("[MoveToCone] PARAM equate_size_to_bumper_hit: %s", equate_size_to_bumper_hit_ ? "TRUE" : "FALSE");
	ROS_INFO("[MoveToCone] PARAM linear_move_meters_per_sec: %7.4f", linear_move_meters_per_sec_);
	ROS_INFO("[MoveToCone] PARAM yaw_turn_radians_per_sec: %7.4f", yaw_turn_radians_per_sec_);

    coneDetectorAnnotatorService_ = nh_.serviceClient<victoria_perception::AnnotateDetectorImage>("/cone_detector/annotate_detector_image");
}

// Capture the lates ConeDetector information
void MoveToCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detected_ = *msg;
	count_ObjectDetector_msgs_received_++;
	if (!last_object_detected_.object_detected) {
		sequential_detection_failures_++;
	} else {
		sequential_detection_failures_ == 0;
	}
}

void MoveToCone::distanceDisplacement1DCb(const victoria_sensor_msgs::DistanceDisplacement1DConstPtr& msg) {
	if (msg->displacement > ((msg->max_value + msg->min_value) / 2.0)) {
		bumper_hit_ = true;
	} else {
		bumper_hit_ = false;
	}
}

// Reset global state so this behavior can be used to solve the next problem.
void MoveToCone::resetGoal() {
	state_ = MOVE_START;
	bumper_hit_ = false;
	last_object_detected_.object_detected = false;
}

StrategyFn::RESULT_T MoveToCone::tick() {
	geometry_msgs::Twist	cmd_vel;		// For sending movement commands to the robot.
	RESULT_T 				result = FATAL;	// Assume fatality in the algorithm.
	std::ostringstream 		ss;				// For sending informations messages.

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_ObjectDetector_msgs_received_ <= 20) {
		// Wait until ConeDetector messages are received.
	    annotator_request_.request.annotation = "UL;FFFFFF;MTC Cone wait";
	    coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	if (state_ == MOVE_START) {
		time_last_saw_cone = ros::Time::now();
		state_ = MOVING_TO_CENTERING_POSITION;
		bumper_hit_ = false;
		return RUNNING;
	}

	if (!last_object_detected_.object_detected) {
		// Failure, lost sight of the cone.
		ros::Time now = ros::Time::now();
		ros::Duration duration_since_last_saw_cone = now - time_last_saw_cone;
		if (duration_since_last_saw_cone.toSec() > 2) {
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
		    annotator_request_.request.annotation = "UL;FFFFFF;MTC FAILED";
		    coneDetectorAnnotatorService_.call(annotator_request_);
			return setGoalResult(FAILED); // No object found.
		} else {
			// Just ignore object detection failure for a while to see if it's a fluke.
			// Note we don't stop motor here. We rely on the motor controller to time out.
		    annotator_request_.request.annotation = "UL;FFFFFF;MTC Lost cone";
		    coneDetectorAnnotatorService_.call(annotator_request_);
			return RUNNING;
		}
	} else {
		time_last_saw_cone = ros::Time::now();
	}

	int image_width = 0;
	int object_x = 0;

	switch (state_) {
	case MOVE_START:
		time_last_saw_cone = ros::Time::now();
		state_ = MOVING_TO_CENTERING_POSITION;
		result = RUNNING;
		break;

	case MOVING_TO_CENTERING_POSITION:
		if (equate_size_to_bumper_hit_) {
			bumper_hit_ |= (last_object_detected_.object_detected) &&
						   (last_object_detected_.object_area > cone_area_for_bumper_hit_);
		}

		if (bumper_hit_) {
			resetGoal();
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << " SUCCESS bumper hit, stopped";
			ss << ", area: " << last_object_detected_.object_area;
			publishStrategyProgress("MoveToCone::tick", ss.str());
			popGoal();
		    annotator_request_.request.annotation = "UL;FFFFFF;MTC SUCCESS";
		    coneDetectorAnnotatorService_.call(annotator_request_);
			return setGoalResult(SUCCESS);
		} 
	
		image_width = last_object_detected_.image_width;
		object_x = last_object_detected_.object_x;
		if (last_object_detected_.image_width > 0) {
			cmd_vel.linear.x = linear_move_meters_per_sec_;
			double degrees_per_pixel = field_of_view_degrees_ / image_width;
			double cone_pixel_from_center = (image_width / 2.0) - last_object_detected_.object_x; // Right of center is negative.
			cmd_vel.angular.z = angles::from_degrees(cone_pixel_from_center *  degrees_per_pixel);
			//cmd_vel.angular.z = ((image_width / 2.0) - last_object_detected_.object_x) > 0 ? yaw_turn_radians_per_sec_ : -yaw_turn_radians_per_sec_;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "Turn, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
			ss << ", area: " << last_object_detected_.object_area;
		    annotator_request_.request.annotation = "UL;FFFFFF;MTC Correct yaw";
		    coneDetectorAnnotatorService_.call(annotator_request_);
		} else {
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "FAULT! image.width <= 0, stopping";
		    annotator_request_.request.annotation = "UL;FFFFFF;MTC Lost cone";
		    coneDetectorAnnotatorService_.call(annotator_request_);
		}			

		result = RUNNING;
		break;

	default:
		ss << "FATAL: invalid state";
		return setGoalResult(FATAL);
	}

	publishStrategyProgress("MoveToCone::tick", ss.str());
	return result;
}

StrategyFn& MoveToCone::singleton() {
    static MoveToCone singleton_;
    return singleton_;
}
