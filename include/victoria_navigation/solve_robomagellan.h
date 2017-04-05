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

#ifndef __SOLVE_ROBOMAGELLAN
#define __SOLVE_ROBOMAGELLAN

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "victoria_navigation/solve_robomagellan.h"
#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"

// A behavior that solves the overall RoboMagellan contest.
//
// The behavior works as follows:
//		* Read a list of GPS waypoints and compute the bearing and heading of the next point of each point (except the last).
//		* Wait until messages are received from the GPS.
//		* The first time this behavior sees this problem, build a list of internal goal waypoints from the yaml listm
//		  computing the bearing and heading of the next point of each point (except the last).
//		  push the next (first) goal waypoint via pushGpsPoint() so that SeekToGps can work on it.
//		  Push the SeekToGps goal as the current goal so it will move the robot near to the waypoint.
//		  Set the state to MOVE_TO_GPS_POINT to away completion of the SeekToGps goal. 
//		* If the state is MOVE_TO_GPS_POINT, the SeekToGps goal must have finished. If there is a cone
//		  at the current waypoint, push the DiscoverCone goal and set the state to FIND_CONE_IN_CAMERA.
//		  Else set the state to ADVANCE_TO_NEXT_POINT.
//		* If the state is FIND_CONE_IN_CAMERA, push the DiscoverCone goal and set the state to MOVE_TO_CONE.
//		* If the state is MOVE_TO_CONE, the DiscoverCone goal must have finished. Push the MoveToCone goal
//		  and set the state to MOVE_FROM_CONE;
//		* If the state is MOVE_FROM_CONE, the MoveToCone goal must have finished. Push the MoveFromCone goal
//		  and set the state to ADVANCE_TO_NEXT_POINT.
//		* If the state is ADVANCE_TO_NEXT_POINT, then if there are no more points indicate SUCCESS and
//		   reset the behavior. Else advance to the next point and set the state to MOVE_TO_GPS_POINT.
//
class SolveRobomMagellan : public StrategyFn {
private:
	enum STATE {
		SETUP,					// Capture initial state.
		MOVE_TO_GPS_POINT,		// Get close to a waypoint.
		FIND_CONE_IN_CAMERA,	// Position the robot so it can see a RoboMagellan cone.
		MOVE_TO_CONE,			// Move to the cone.
		MOVE_FROM_CONE,			// Move away from the cone, setting up to move to the next waypoint.
		ADVANCE_TO_NEXT_POINT	// Advance to the next waypoint.
	};

	// Parameters.
	std::string fix_topic_name_;				// Topic name containing fix message.
	std::string waypoint_yaml_path_;			// Path to yaml file containing waypoints.

	// Subscribers.
	ros::Subscriber fix_sub_;

	// Algorithm variables.
	ros::ServiceClient coneDetectorAnnotatorService_;	// For annotating the cone detector image.
	std::vector<GPS_POINT> gps_points_;				// Ordered list of waypoints to traverse.
	long int index_next_point_to_seek_;			// Index into list of GPS points to seek to next.
	victoria_perception::AnnotateDetectorImage annotator_request_;	// The annotation request.
	STATE state_;								// State of state machine.
	YAML::Node waypoints_;						// Waypoints in yaml format.
	
	// ROS node handle.
	ros::NodeHandle nh_;

	// Process one Fix topic message.
	long int count_Fix_msgs_received_;
	sensor_msgs::NavSatFix last_Fix_msg_;
	void fixCb(const sensor_msgs::NavSatFixConstPtr& msg);

	static double degrees(double x) { return x * 180.0 / M_PI; }
	static double radians(double x) { return x * M_PI / 180.0; }

 	// Calculate bearing between two GPS points--accurate for distances for RoboMagellan.
	double bearing(sensor_msgs::NavSatFix from, sensor_msgs::NavSatFix to) {
	    double lat1 = radians(from.latitude);
	    double lon1 = radians(from.longitude);
	    double lat2 = radians(to.latitude);
	    double lon2 = radians(to.longitude);
	    double dLon = lon2 - lon1;
	 
	    double y = sin(dLon) * cos(lat2);
	    double x = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dLon));
	 
	    return degrees(atan2(y, x)); 
	}
	 
	 
 	// Calculate distance between two GPS points--accurate for distances for RoboMagellan.
	double distance(sensor_msgs::NavSatFix from, sensor_msgs::NavSatFix to) {
	    double lat1 = radians(from.latitude);
	    double lon1 = radians(from.longitude);
	    double lat2 = radians(to.latitude);
	    double lon2 = radians(to.longitude);
	    double dLat = lat2 - lat1;
	    double dLon = lon2 - lon1;
	 
	    double a = (sin(dLat / 2.0) * sin(dLat / 2.0)) + 
	               ((cos(lat1) * cos(lat2)) *
	                (sin(dLon / 2.0) * sin(dLon / 2.0)));
	    double c = 2.0 * atan2(sqrt(a), sqrt(1 - a));
	    
	    return 6371000.0 * c;
	}

	// Normalize an Euler angle into [0..360).
	double normalizeEuler(double yaw_degrees);

	// Reset goal. After this, someone must request the goal again and it will start over.
	void resetGoal();

	// Singleton pattern.
	SolveRobomMagellan();
	SolveRobomMagellan(SolveRobomMagellan const&) {};
	SolveRobomMagellan& operator=(SolveRobomMagellan const&) {};

public:
	RESULT_T tick();

	std::string goalName() { return "/strategy/solve_robomagellan"; }

	std::string name() { return std::string("SolveRobomMagellan"); };

	static SolveRobomMagellan& singleton();

	std::string stateName(STATE state) {
		switch (state) {
			case SETUP:					return "SETUP";
			case MOVE_TO_GPS_POINT:		return "MOVE_TO_GPS_POINT";
			case FIND_CONE_IN_CAMERA:	return "FIND_CONE_IN_CAMERA";
			case MOVE_TO_CONE:			return "MOVE_TO_CONE";
			case MOVE_FROM_CONE:		return "MOVE_FROM_CONE";
			case ADVANCE_TO_NEXT_POINT:	return "ADVANCE_TO_NEXT_POINT";
			default:					return "!!UNKNOWN!!";
		}
	}

};

#endif