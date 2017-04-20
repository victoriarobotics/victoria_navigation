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
#include <unistd.h>

#include "victoria_navigation/discover_cone.h"
#include "victoria_navigation/move_from_cone.h"
#include "victoria_navigation/move_to_cone.h"
#include "victoria_navigation/seek_to_gps.h"
#include "victoria_navigation/solve_robomagellan.h"

SolveRoboMagellan::SolveRoboMagellan() :
	index_next_point_to_seek_(0),
	move_to_cone_recovery_count_(0),
	state_(SETUP)
{
	assert(ros::param::get("~fix_topic_name", fix_topic_name_));
	assert(ros::param::get("~waypoint_yaml_path", waypoint_yaml_path_));
	
	assert(fix_sub_ = nh_.subscribe(fix_topic_name_, 1, &SolveRoboMagellan::fixCb));

	ROS_INFO("[SolveRoboMagellan] PARAM fix_topic_name: %s", fix_topic_name_.c_str());
	ROS_INFO("[SolveRoboMagellan] PARAM waypoint_yaml_path: %s", waypoint_yaml_path_.c_str());

	createGpsPointSet(waypoint_yaml_path_);

    coneDetectorAnnotatorService_ = nh_.serviceClient<victoria_perception::AnnotateDetectorImage>("/cone_detector/annotate_detector_image");
}

void SolveRoboMagellan::fixCb(const sensor_msgs::NavSatFixConstPtr& msg) {
	if (g_count_Fix_msgs_received_ == 0) SolveRoboMagellan::g_first_Fix_msg_ = *msg;
	SolveRoboMagellan::g_last_Fix_msg_ = *msg;
	SolveRoboMagellan::g_count_Fix_msgs_received_++;
}

void SolveRoboMagellan::resetGoal() {
	index_next_point_to_seek_ = 0;
	state_ = SETUP;
}

void SolveRoboMagellan::createGpsPointSet(std::string waypoint_yaml_path) {
	while (g_count_Fix_msgs_received_ <= 0) { ros::spinOnce(); }

	ROS_INFO("[SolveRoboMagellan::createGpsPointSet] Robot is located at latitude: %11.7f, longitude: %11.7f", g_last_Fix_msg_.latitude, g_last_Fix_msg_.longitude);

	int point_number = 0;
	std::vector<GPS_POINT> gps_points;				// Ordered list of waypoints to traverse.
	YAML::Node waypoints = YAML::LoadFile(waypoint_yaml_path);
	sensor_msgs::NavSatFix previous_point = g_first_Fix_msg_;  // Implied starting point is the current GPS fix.

    if (waypoints["gps_points"].Type() != YAML::NodeType::Sequence) {
        ROS_ERROR("[SolveRoboMagellan::createGpsPointSet] Unable to load yaml file: %s", waypoint_yaml_path.c_str());
        return;
    } else {
        ROS_INFO("[SolveRoboMagellan::createGpsPointSet Number of GPS points in the set: %ld", waypoints["gps_points"].size());
    }

	for (YAML::const_iterator yamlPoint = waypoints["gps_points"].begin(); yamlPoint != waypoints["gps_points"].end(); yamlPoint++) {
		// Compute the heading and distance from one point to the next for all points.
		YAML::Node n = *yamlPoint;
		GPS_POINT gps_point;
		if (n["is_home"] && n["is_home"].as<bool>()) {
			g_first_Fix_msg_.latitude = n["latitude"].as<double>();
			g_first_Fix_msg_.longitude = n["longitude"].as<double>();
			previous_point.latitude = g_first_Fix_msg_.latitude;
			previous_point.longitude = g_first_Fix_msg_.longitude;
			continue;
		}

		gps_point.latitude = n["latitude"].as<double>();
		gps_point.longitude = n["longitude"].as<double>();
		gps_point.has_cone = n["has_cone"].as<bool>();

		sensor_msgs::NavSatFix yaml_point_as_fix;
		yaml_point_as_fix.latitude = gps_point.latitude;
		yaml_point_as_fix.longitude = gps_point.longitude;
		double point_bearing_gps = headingInGpsCoordinates(previous_point, yaml_point_as_fix);
		double point_distance = greatCircleDistance(previous_point, yaml_point_as_fix);
		double point_bearing_odom = headingInGpsCoordinates(g_first_Fix_msg_, yaml_point_as_fix);
		double point_distance_odom = greatCircleDistance(g_first_Fix_msg_, yaml_point_as_fix);
		double goal_yaw = angles::normalize_angle((M_PI / 2.0) - point_bearing_gps);

		gps_point.bearing = goal_yaw;
		gps_point.x = point_distance_odom * cos(goal_yaw);
		gps_point.y = point_distance_odom * sin(goal_yaw);
		gps_point.distance = point_distance;
		gps_point.point_number = point_number;
		gps_points_.push_back(gps_point);
		ROS_INFO("[SolveRoboMagellan::createGpsPointSet] point: %d"
				  ", FROM lat: %11.7f"
				  ", lon: %11.7f"
				  ", TO lat: %11.7f"
				  ", lon: %11.7f"
				  ", distance: %7.1f"
				  ", ros heading/d from prev: %7.1f"
				  ", GPS heading/d from prev: %7.1f"
				  ", x: %7.3f"
				  ", y: %7.3f"
				  ", has_cone: %s",
				  point_number++,
				  previous_point.latitude,
				  previous_point.longitude,
				  gps_point.latitude,
				  gps_point.longitude,
				  gps_point.distance,
				  angles::to_degrees(gps_point.bearing),
				  angles::to_degrees(point_bearing_gps),
				  gps_point.x,
				  gps_point.y,
				  gps_point.has_cone ? "TRUE" : "FALSE"
				  );
		previous_point = yaml_point_as_fix;
	}
}

StrategyFn::RESULT_T SolveRoboMagellan::tick() {
	RESULT_T 					result = FATAL;
	std::ostringstream 				ss;

 	if (StrategyFn::currentGoalName() != goalName()) {
		// This is not a problem the behavior can solve.
		return INACTIVE;
	}

	if (g_count_Fix_msgs_received_ <= 0) {
		// Wait until Fix messages are received.
        annotator_request_.request.annotation = "LL;FFFFFF;Wait for /fix";
        coneDetectorAnnotatorService_.call(annotator_request_);
		return RUNNING;
	}

	switch (state_) {
	case SETUP:
		// Capture the initial state.
		index_next_point_to_seek_ = 0;
		pushGpsPoint(gps_points_[index_next_point_to_seek_]);
		StrategyFn::pushGoal(SeekToGps::singleton().goalName(), "");
		state_ = MOVE_TO_GPS_POINT;
		ss << "Setup complete, seeking to first point.";
		result = RUNNING;

        annotator_request_.request.annotation = "LL;FFFFFF;Moving to point";
        coneDetectorAnnotatorService_.call(annotator_request_);

        break;

	case MOVE_TO_GPS_POINT:
		result = doMoveToGpsPoint(ss);
		break;

	case FIND_CONE_IN_CAMERA:
		result = doFindConeInCamera(ss);
		break;

	case MOVE_TO_CONE:
		result = doMoveToCone(ss);
		break;

	case MOVE_FROM_CONE:
		result = doMoveFromCone(ss);
		break;

	case ADVANCE_TO_NEXT_POINT:
		result = doAdvanceToNextPoint(ss);
		break;

	case MTCR_MOVE_FROM_CONE:
		result = doMtcrMoveFromCone(ss);
		break;

	case MTCR_DISCOVER_CONE:
		result = doMtcrDiscoverCone(ss);
		break;

	default:
		ROS_ERROR("SolveRoboMagellan::tick] INVALID state_ value");

        annotator_request_.request.annotation = "LL;FFFFFF;FATAL invalid state value";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		return setGoalResult(FATAL);
		break;
	}

	publishStrategyProgress("SolveRoboMagellan::tick", ss.str());
	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doMoveToGpsPoint(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;

	if (lastGoalResult() == SUCCESS) {
		ROS_INFO("[SolveRoboMagellan::doMoveToGpsPoint] succeeded in SeekToGps for point: %ld", index_next_point_to_seek_);
		out_ss << "SeekToGps successful to point: ";
		out_ss << index_next_point_to_seek_;
		if (!gps_points_[index_next_point_to_seek_].has_cone) {
			state_ = ADVANCE_TO_NEXT_POINT;
			out_ss << ", no cone here, advancing to next point.";

	        annotator_request_.request.annotation = "LL;FFFFFF;At point, move on";
	        coneDetectorAnnotatorService_.call(annotator_request_);
		} else {
			StrategyFn::pushGoal(DiscoverCone::singleton().goalName(), "");
			state_ = FIND_CONE_IN_CAMERA;
			out_ss << ", discovering cone.";

	        annotator_request_.request.annotation = "LL;FFFFFF;At point, find cone";
	        coneDetectorAnnotatorService_.call(annotator_request_);
		}

		result = RUNNING;
	} else {
		ROS_ERROR("[SolveRoboMagellan::doMoveToGpsPoint] NO BACKTRACK STRATEGY FOR MOVE_TO_GPS_POINT. lastGoalResult: %d", lastGoalResult());
        annotator_request_.request.annotation = "LL;0000FF;NO BT for MOVE_TO_GPS_POINT";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = setGoalResult(FATAL);
	}

	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doFindConeInCamera(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;
	if (lastGoalResult() == SUCCESS) {
		ROS_INFO("[SolveRoboMagellan::doFindConeInCamera] succeeded in DiscoverCone for point: %ld", index_next_point_to_seek_);
		move_to_cone_recovery_count_ = 0;
		StrategyFn::pushGoal(MoveToCone::singleton().goalName(), "");
		state_ = MOVE_TO_CONE;
		out_ss << "DiscoverCone successful for point: ";
		out_ss << index_next_point_to_seek_;
		out_ss << ", moving towards cone.";

        annotator_request_.request.annotation = "LL;FFFFFF;Found cone, move to it";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = RUNNING;
	} else {
		ROS_ERROR("[SolveRoboMagellan::doFindConeInCamera] NO BACKTRACKING FOR FIND_CONE_IN_CAMERA");
        annotator_request_.request.annotation = "LL;0000FF;NO BT for FIND_CONE_IN_CAMERA";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = setGoalResult(FATAL);
	}

	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doMoveToCone(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;

	if (lastGoalResult() == SUCCESS) {
		move_to_cone_recovery_count_ = 0;
		ROS_INFO("[SolveRoboMagellan::doMoveToCone] succeeded in MoveToCone for point: %ld", index_next_point_to_seek_);
		StrategyFn::pushGoal(MoveFromCone::singleton().goalName(), "");
		state_ = MOVE_FROM_CONE;
		out_ss << "MoveToCone successful for point: " << index_next_point_to_seek_ << ", moving from cone.";

        annotator_request_.request.annotation = "LL;FFFFFF;Touched cone, move away";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = RUNNING;
	} else {
		if (move_to_cone_recovery_count_++ < 2) {
			// Assume MoveToCone failed because it lost sight of the cone.
			// Backup a bit, look for the cone again, and retry.
			ROS_INFO("[SolveRoboMagellan::tick] MoveToCone failed for point: %ld, initiating recovery attempt: %d",
					 index_next_point_to_seek_,
					 move_to_cone_recovery_count_);
			StrategyFn::pushGoal(MoveFromCone::singleton().goalName(), "");
			state_ = MTCR_MOVE_FROM_CONE;
			out_ss << "MoveToCone failed for point: " << index_next_point_to_seek_;
			out_ss << ", initiating recovery attempt: " << move_to_cone_recovery_count_;
	        annotator_request_.request.annotation = "LL;FFFFFF;MoveToCone failed, move away";
	        coneDetectorAnnotatorService_.call(annotator_request_);
			result = RUNNING;
		} else {
			move_to_cone_recovery_count_ = 0;
			ROS_ERROR("[SolveRoboMagellan::doMoveToCone] No further recovery available for failed MoveToCone, moving on");
	        annotator_request_.request.annotation = "LL;0000FF;NO MORE RECOVERY FOR MoveToCone";;
	        coneDetectorAnnotatorService_.call(annotator_request_);
			result = setGoalResult(SUCCESS);
		}
	}

	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doMtcrMoveFromCone(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;

	if (lastGoalResult() == SUCCESS) {
		ROS_INFO("[SolveRoboMagellan::doMtcrMoveFromCone] succeeded in MoveFromCone in MTC recovery");
		StrategyFn::pushGoal(DiscoverCone::singleton().goalName(), "");
		state_ = MTCR_DISCOVER_CONE;
		out_ss << "MTC recovery MoveFromCone successful for point: ";
		out_ss << index_next_point_to_seek_;
		out_ss << ", do DiscoverCone.";

        annotator_request_.request.annotation = "LL;FFFFFF;MTCR_MOVE_FROM_CONE, do DiscoverCone";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = RUNNING;
	} else {
		// Unable to move from the cone during MoveToCone recovery.
		ROS_ERROR("[SolveRoboMagellan::doMtcrMoveFromCone] FAILED during MoveFromCone, giving up");
		annotator_request_.request.annotation = "LL;0000FF;NO MORE RECOVERY FOR MtcMoveFromCone";
		coneDetectorAnnotatorService_.call(annotator_request_);
		result = setGoalResult(FATAL);
	}

	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doMtcrDiscoverCone(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;

	if (lastGoalResult() == SUCCESS) {
		ROS_INFO("[SolveRoboMagellan::doMtcrMoveFromCone] succeeded in DiscoverCone in MTC recovery");
		StrategyFn::pushGoal(MoveToCone::singleton().goalName(), "");
		state_ = MOVE_TO_CONE;;
		out_ss << "MTC recovery MoveFromCone successful for point: ";
		out_ss << index_next_point_to_seek_;
		out_ss << ", retry MoveToCone.";

        annotator_request_.request.annotation = "LL;FFFFFF;MTCR_MOVE_FROM_CONE, retry MoveToCone";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = RUNNING;
	} else {
		// Unable to move from the cone during MoveToCone recovery.
		ROS_ERROR("[SolveRoboMagellan::doMtcrMoveFromCone] FAILED during DiscoverCone, giving up");
		annotator_request_.request.annotation = "LL;0000FF;NO MORE RECOVERY FOR MtcMoveFromCone";
		coneDetectorAnnotatorService_.call(annotator_request_);
		result = setGoalResult(FATAL);
	}

	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doMoveFromCone(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;
	if (lastGoalResult() == SUCCESS) {
		ROS_INFO("[SolveRoboMagellan::doMoveFromCone] succeeded in MoveFromCone for point: %ld", index_next_point_to_seek_);
		state_ = ADVANCE_TO_NEXT_POINT;
		out_ss << "MoveFromCone successful for point: ";
		out_ss << index_next_point_to_seek_;
		out_ss << ", advancing to next point.";
		state_ = ADVANCE_TO_NEXT_POINT;

        annotator_request_.request.annotation = "LL;FFFFFF;Backed away, onto next point";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = RUNNING;
	} else {
		ROS_ERROR("[SolveRoboMagellan::doMoveFromCone] NO BACKTRACKING FOR MOVE_FROM_CONE");
        annotator_request_.request.annotation = "LL;00FFFF;NO BT for MOVE_FROM_CONE";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = setGoalResult(FATAL);
	}

	return result;
}

StrategyFn::RESULT_T SolveRoboMagellan::doAdvanceToNextPoint(std::ostringstream& out_ss) {
	RESULT_T result = FATAL;

	index_next_point_to_seek_++;
	if (index_next_point_to_seek_ >= gps_points_.size()) {
		resetGoal();
		popGoal();
		out_ss << "Completed tour of all points. SUCCESS";

        annotator_request_.request.annotation = "LL;FFFFFF;SUCCESSFUL RoboMagellan";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = setGoalResult(SUCCESS);
	} else {
		pushGpsPoint(gps_points_[index_next_point_to_seek_]);
		StrategyFn::pushGoal(SeekToGps::singleton().goalName(), "");
		state_ = MOVE_TO_GPS_POINT;
		out_ss << "Advancing to point: ";
		out_ss << index_next_point_to_seek_;

        annotator_request_.request.annotation = "LL;FFFFFF;Setting up for next point";;
        coneDetectorAnnotatorService_.call(annotator_request_);
		result = RUNNING;
	}

	return result;
}

StrategyFn& SolveRoboMagellan::singleton() {
    static SolveRoboMagellan singleton_;
    return singleton_;
}

std::vector<SolveRoboMagellan::GPS_POINT> SolveRoboMagellan::gps_points_;
long int SolveRoboMagellan::g_count_Fix_msgs_received_;
sensor_msgs::NavSatFix SolveRoboMagellan::g_first_Fix_msg_;
sensor_msgs::NavSatFix SolveRoboMagellan::g_last_Fix_msg_;
