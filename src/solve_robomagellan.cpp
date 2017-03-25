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

	ROS_INFO("[SolveRobomMagellan] PARAM fix_topic_name_: %s", fix_topic_name_.c_str());
	ROS_INFO("[SolveRobomMagellan] PARAM waypoint_yaml_path: %s", waypoint_yaml_path_.c_str());

	waypoints_ = YAML::LoadFile(waypoint_yaml_path_);
	if (waypoints_["gps_points"].Type() != YAML::NodeType::Sequence) {
		ROS_ERROR("[SolveRobomMagellan] Unable to load yaml file: %s", waypoint_yaml_path_.c_str());
	} else {
		ROS_INFO("[SolveRobomMagellan] Number of GPS points in the set: %ld", waypoints_["gps_points"].size());
	}
}

void SolveRobomMagellan::fixCb(const sensor_msgs::NavSatFixConstPtr& msg) {
	last_Fix_msg_ = *msg;
	count_Fix_msgs_received_++;
}

double SolveRobomMagellan::normalizeEuler(double yaw_degrees) {
	double result = yaw_degrees;
	while (result > 360) result -= 360;
	while (result < 0) result += 360;
	return result;
}

void SolveRobomMagellan::resetGoal() {
	index_next_point_to_seek_ = 0;
	state_ = SETUP;
}

StrategyFn::RESULT_T SolveRobomMagellan::tick() {
	RESULT_T 					result = FATAL;
	ostringstream 				ss;

	if (StrategyFn::currentGoalName() != goalName()) { //#####
		ss << "FAILED, goal not active: ";
		ss << goalName();
		publishStrategyProgress("SolveRobomMagellan::tick", ss.str());
		return setGoalResult(FAILED);
	}

	if (count_Fix_msgs_received_ <= 0) {
		ss << "No Fix messages received";
		publishStrategyProgress("SolveRobomMagellan::tick", ss.str());
		return setGoalResult(RUNNING); // No data yet.
	}

	if (state_ == SETUP) {
		int point_number = 0;
		sensor_msgs::NavSatFix previous_point = last_Fix_msg_;
		for (YAML::const_iterator yamlPoint = waypoints_["gps_points"].begin(); yamlPoint != waypoints_["gps_points"].end(); yamlPoint++) {
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
		pushGpsPoint(gps_points_[0]);
		StrategyFn::pushGoal(SeekToGps::singleton().goalName(), "0");
		state_ = MOVE_TO_GPS_POINT;
		return setGoalResult(RUNNING);
	} else if (state_ == MOVE_TO_GPS_POINT) {
		if (lastGoalResult() == SUCCESS) {
			ROS_INFO("### MOVE_TO_GPS_POINT");
			popGoal();
			resetGoal();
			return setGoalResult(SUCCESS);
		} else {
			ROS_ERROR("NO BACKTRACK STRATEGY FOR MOVE_TO_GPS_POINT");
			return setGoalResult(FATAL);
		}
	}
	publishStrategyProgress("SolveRobomMagellan::tick", ss.str());

	return setGoalResult(result);
}

SolveRobomMagellan& SolveRobomMagellan::singleton() {
    static SolveRobomMagellan singleton_;
    return singleton_;
}
