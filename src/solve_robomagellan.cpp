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

// Steps:
// * Read list of points.
// * Compute positions of points.
// * Push points to point_stack.
// * State: SEEK_TO_GPS
// *	if point_stack is empty
// *		last_goal_result = SUCCESS
// *		return SUCCESS
// *	push goal seek_to_gps
// *	state = MOVING_TO_GPS_POINT
// * 	return RUNNING
// * State: MOVING_TO_GPS_POINT
// *	if last_goal_result (i.e., seek_to_gps) = SUCCESS
// *		if has_cone
// *			push goal discover_cone
// *			state = LOOKING_FOR_CONE
// *		else
// *			pop point_stack
// *			state = SEEK_TO_GPS 	// Advance to next point
// *		return RUNNING
// *	else TODO -- handle seek_to_gps failure
// * State: LOOKING_FOR_CONE
// *	if last_goal_result (i.e., move_to_cone) = SUCCESS
// *		pop point_stack
// * 		TODO advance away from cone
// *		state = SEEK_TO_GPS 		// Advance to next point
// *	else TODO -- handle move_to_cone failure.

#include <cassert>
#include <ros/ros.h>
#include <unistd.h>

#include "victoria_navigation/discover_cone.h"
#include "victoria_navigation/move_to_cone.h"
#include "victoria_navigation/seek_to_gps.h"
#include "victoria_navigation/solve_robomagellan.h"

SolveRobomMagellan::SolveRobomMagellan() :
	index_next_point_to_seek_(0),
	state_(SETUP)
{
	assert(ros::param::get("~fix_topic_name", fix_topic_name_));
	assert(ros::param::get("~waypoint_yaml_path", waypoint_yaml_path_));
	
	assert(fix_sub_ = nh_.subscribe(fix_topic_name_, 1, &SolveRobomMagellan::fixCb, this));

	ROS_INFO("[SolveRobomMagellan] PARAM fix_topic_name: %s", fix_topic_name_.c_str());
	ROS_INFO("[SolveRobomMagellan] PARAM waypoint_yaml_path: %s", waypoint_yaml_path_.c_str());

	waypoints_ = YAML::LoadFile(waypoint_yaml_path_);
	if (waypoints_["gps_points"].Type() != YAML::NodeType::Sequence) {
		ROS_ERROR("[SolveRobomMagellan] Unable to load yaml file: %s", waypoint_yaml_path_.c_str());
	} else {
		ROS_INFO("[SolveRobomMagellan] Number of GPS points in the set: %ld", waypoints_["gps_points"].size());
	}
}

// Capture the lates Fix information.
void SolveRobomMagellan::fixCb(const sensor_msgs::NavSatFixConstPtr& msg) {
	last_Fix_msg_ = *msg;
	count_Fix_msgs_received_++;
}

// Convert an Euler angle into a range of [0 .. 360).
double SolveRobomMagellan::normalizeEuler(double yaw_degrees) {
	double result = yaw_degrees;
	while (result > 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

// Reset global state so this behavior can be used to solve the next problem.
void SolveRobomMagellan::resetGoal() {
	index_next_point_to_seek_ = 0;
	state_ = SETUP;
}

StrategyFn::RESULT_T SolveRobomMagellan::tick() {
	RESULT_T 					result = FATAL;
	ostringstream 				ss;

	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (count_Fix_msgs_received_ <= 0) {
		// Wait until Fix messages are received.
		return RUNNING;
	}

	if (state_ == SETUP) {
		// Capture the initial state.
		int point_number = 0;
		sensor_msgs::NavSatFix previous_point = last_Fix_msg_;  // Implied starting point is the current GPS fix.
		for (YAML::const_iterator yamlPoint = waypoints_["gps_points"].begin(); yamlPoint != waypoints_["gps_points"].end(); yamlPoint++) {
			// Compute the heading and distance from one point to the next for all points.
			YAML::Node n = *yamlPoint;
			GPS_POINT gps_point;
			gps_point.latitude = n["latitude"].as<double>();
			gps_point.longitude = n["longitude"].as<double>();
			gps_point.has_cone = n["has_cone"].as<bool>();

			sensor_msgs::NavSatFix yaml_point_as_fix;
			yaml_point_as_fix.latitude = gps_point.latitude;
			yaml_point_as_fix.longitude = gps_point.longitude;
			double point_bearing_gps_degrees = bearing(previous_point, yaml_point_as_fix);
			double point_distance = distance(previous_point, yaml_point_as_fix);
			double goal_yaw_degrees_ = normalizeEuler(90 - point_bearing_gps_degrees);;

			gps_point.bearing = goal_yaw_degrees_;
			gps_point.x = point_distance * cos(radians(goal_yaw_degrees_));
			gps_point.y = point_distance * sin(radians(goal_yaw_degrees_));
			gps_point.distance = point_distance;
			gps_points_.push_back(gps_point);
			ROS_INFO_COND(do_debug_strategy_,
						  "[SolveRobomMagellan::tick] point: %d"
						  ", FROM lat: %11.7f"
						  ", lon: %11.7f"
						  ", TO lat: %11.7f"
						  ", lon: %11.7f"
						  ", distance: %7.4f"
						  ", GPS heading: %7.4f"
						  ", ROS heading: %7.4f"
						  ", x: %11.7f"
						  ", y: %11.7f"
						  ", has_cone: %s",
						  point_number++,
						  previous_point.latitude,
						  previous_point.longitude,
						  gps_point.latitude,
						  gps_point.longitude,
						  gps_point.distance,
						  point_bearing_gps_degrees,
						  gps_point.bearing,
						  gps_point.x,
						  gps_point.y,
						  gps_point.has_cone ? "TRUE" : "FALSE"
						  );
			previous_point = yaml_point_as_fix;
		}

		index_next_point_to_seek_ = 0;
		pushGpsPoint(gps_points_[index_next_point_to_seek_]);
		StrategyFn::pushGoal(SeekToGps::singleton().goalName(), "0");
		state_ = MOVE_TO_GPS_POINT;
		ss << "Setup complete, seeking to first point.";
		result = RUNNING;
	} else if (state_ == MOVE_TO_GPS_POINT) {
		if (lastGoalResult() == SUCCESS) {
			ROS_INFO("[SolveRobomMagellan::tick] succeeded in SeekToGps for point: %ld", index_next_point_to_seek_);
			ss << "SeekToGps successful to point: ";
			ss << index_next_point_to_seek_;
			if (!gps_points_[index_next_point_to_seek_].has_cone) {
				state_ = ADVANCE_TO_NEXT_POINT;
				ss << ", no cone here, advancing to next point.";
			} else {
				StrategyFn::pushGoal(DiscoverCone::singleton().goalName(), "0");
				state_ = FIND_CONE_IN_CAMERA;
				ss << ", discovering cone.";
			}

			result = RUNNING;
		} else {
			ROS_ERROR("NO BACKTRACK STRATEGY FOR MOVE_TO_GPS_POINT. lastGoalResult: %d", lastGoalResult());
			return setGoalResult(FATAL);
		}
	} else if (state_ == FIND_CONE_IN_CAMERA) {
		if (lastGoalResult() == SUCCESS) {
			ROS_INFO("[SolveRobomMagellan::tick] succeeded in DiscoverCone for point: %ld", index_next_point_to_seek_);
			StrategyFn::pushGoal(DiscoverCone::singleton().goalName(), "0");
			state_ = MOVE_TO_CONE;
			ss << "DiscoverCone successful for point: ";
			ss << index_next_point_to_seek_;
			ss << ", moving towards cone.";
			result = RUNNING;
		} else {
			ROS_ERROR("NO BACKTRACKING FOR FIND_CONE_IN_CAMERA");
			return setGoalResult(FATAL);
		}
	} else if (state_ == MOVE_TO_CONE) {
		if (lastGoalResult() == SUCCESS) {
			ROS_INFO("[SolveRobomMagellan::tick] succeeded in MoveToCone for point: %ld", index_next_point_to_seek_);
			//##### StrategyFn::pushGoal(MoveToCone::singleton().goalName(), "0");
			state_ = MOVE_FROM_CONE;
			ss << "MoveToCone successful for point: ";
			ss << index_next_point_to_seek_;
			ss << ", moving from cone.";
			result = RUNNING;
		} else {
			ROS_ERROR("NO BACKTRACKING FOR MOVE_TO_CONE");
			return setGoalResult(FATAL);
		}
	} else if (state_ == MOVE_FROM_CONE) {
		if (lastGoalResult() == SUCCESS) {
			ROS_INFO("[SolveRobomMagellan::tick] succeeded in MoveFromCone for point: %ld", index_next_point_to_seek_);
			state_ = ADVANCE_TO_NEXT_POINT;
			ss << "MoveFromCone successful for point: ";
			ss << index_next_point_to_seek_;
			ss << ", advancing to next point.";
			state_ = ADVANCE_TO_NEXT_POINT;
			result = RUNNING;
		} else {
			ROS_ERROR("NO BACKTRACKING FOR MOVE_FROM_CONE");
			return setGoalResult(FATAL);
		}
	} else if (state_ == ADVANCE_TO_NEXT_POINT) {
		index_next_point_to_seek_++;
		if (index_next_point_to_seek_ >= gps_points_.size()) {
			resetGoal();
			popGoal();
			ss << "Completed tour of all points. SUCCESS";
			result = setGoalResult(SUCCESS);
		} else {
			pushGpsPoint(gps_points_[index_next_point_to_seek_]);
			StrategyFn::pushGoal(SeekToGps::singleton().goalName(), "0");
			state_ = MOVE_TO_GPS_POINT;
			ss << "Advancing to point: ";
			ss << index_next_point_to_seek_;
			result = RUNNING;
		}
	} else {
		ROS_ERROR("INVALID state_ value");
		return setGoalResult(FATAL);
	}

	publishStrategyProgress("SolveRobomMagellan::tick", ss.str());
	return result;
}

SolveRobomMagellan& SolveRobomMagellan::singleton() {
    static SolveRobomMagellan singleton_;
    return singleton_;
}
