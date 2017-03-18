#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/strategy_context.h"
#include "victoria_navigation/move_to_cone.h"
#include "victoria_perception/ObjectDetector.h"

MoveToCone::MoveToCone() :
	object_detections_received_(0),
	state_(kMOVING_TO_CENTERING_POSITION),
	strategy_context_(StrategyContext::singleton())
{
	ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_);
	ros::param::get("~cone_detector_topic_name", cone_detector_topic_name_);
	
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);
	current_strategy_pub_ = nh_.advertise<std_msgs::String>("current_strategy", 1, true /* latched */);
	strategy_status_publisher_ = nh_.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);

	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &MoveToCone::coneDetectorCb, this);

	ROS_INFO("[MoveToCone] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_INFO("[MoveToCone] PARAM cone_detector_topic_name: %s", cone_detector_topic_name_.c_str());
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
		ROS_INFO("[MoveToCone] strategy: %s", strategy.c_str());
	}
}

StrategyFn::RESULT_T MoveToCone::tick() {
	geometry_msgs::Twist		cmd_vel;
	actionlib_msgs::GoalStatus 	goal_status;
	RESULT_T 					result = FATAL;
	ostringstream 				ss;

	if (object_detections_received_ <= 0) {
		ROS_INFO("[MoveToCone::tick] FAILED: no cone_detector messages received");
		return FAILED; // No data yet.
	}

	if (!last_object_detected_.object_detected) {
		// TODO -- rotate until found.
		ROS_INFO("[MoveToCone::tick] FAILED: no object detected");
		return FAILED; // No cone found.
	}

	if (strategy_context_.blackboard["needToMoveToCone"].compare("T") != 0) {
		ROS_INFO("[MoveToCone::tick] FAILED: needToMoveToCone not active");
		return FAILED;
	}

	goal_status.goal_id.stamp = ros::Time::now();
	goal_status.goal_id.id = "MoveToCone";
	goal_status.status = actionlib_msgs::GoalStatus::ACTIVE;

	if (state_ == kMOVING_TO_CENTERING_POSITION) {
		if (last_object_detected_.object_area > 100000) {
			//### Fake test to stop when close.
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "[" << goal_status.goal_id.stamp << " MoveToCone::tick]";
			ss << " close, STOP";
			ss << ", area: " << last_object_detected_.object_area;
			strategy_context_.blackboard["needToMoveToCone"] = "F";
		}

		result = RUNNING;
		int image_width = last_object_detected_.image_width;
		int object_x = last_object_detected_.object_x;
		if (abs((image_width / 2) - object_x) < (image_width / 40)) {
			// Heading generally in the right direction.
			cmd_vel.linear.x = 0.15; //### Arbitrary.
			cmd_vel.angular.z = 0;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "[" << goal_status.goal_id.stamp << " MoveToCone::tick]";
			ss << " go straight, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
			ss << ", area: " << last_object_detected_.object_area;
		} else {
			// Turn towards cone.
			cmd_vel.linear.x = 0.2; //### Arbitrary.
			cmd_vel.angular.z = ((image_width / 2.0) - last_object_detected_.object_x) / last_object_detected_.image_width;
			cmd_vel_pub_.publish(cmd_vel);
			ss << "[" << goal_status.goal_id.stamp << " MoveToCone::tick]";
			ss << " turn, linear.x: " << cmd_vel.linear.x << ", angular.z: " << cmd_vel.angular.z;
			ss << ", area: " << last_object_detected_.object_area;
		}
	} else if (state_ == kMOVING_TO_TOUCH) {
		ROS_INFO("[MoveToCone::tick] FAILED: Unimplemented state kMOVING_TO_TOUCH");
		return FAILED;
	} else {
		ROS_ERROR("[MoveToCone::tick] FAILED: invalid state");
		return FAILED;
	}

	goal_status.text = ss.str();
	strategy_status_publisher_.publish(goal_status);
	ROS_INFO("[MoveToCone::tick] %s", ss.str().c_str());

	return result;
}

MoveToCone& MoveToCone::singleton() {
    static MoveToCone singleton_;
    return singleton_;
}