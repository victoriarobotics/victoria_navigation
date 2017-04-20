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

#ifndef __VICTORIA_NAVIGATION_SOLVE_ROBOMAGELLAN
#define __VICTORIA_NAVIGATION_SOLVE_ROBOMAGELLAN

#include <ros/ros.h>
#include <angles/angles.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "victoria_navigation/solve_robomagellan.h"
#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"
//* A SolveRoboMagellan class.
/**
* A problem solver that solves the overall RoboMagellan contest.
*
* The problem solver attempts to solve the whole RoboMagellan problem—traversing to a list of waypoints
* and ultimately touching the last cone.
*
* The singleton instance of the problem solver, during construction, parses the list of GPS waypoints
* provided in the YAML file pointed at by the waypoint_yaml_path parameter.
*
* The first time the problem solver is invoked to achieve the goal, it begins by waiting on Fix messages. 
* When at least one message is received, the problem solver proceeds.
*
* The problem solver is a state machine that pushes subgoals to advance to the next waypoint in the series
* and, if that waypoint is supposed to have a cone at that location, it also attempts to touch the cone.
* So the problem solver iterates over each point in the waypoint list and for each point it sequentially:
*    * Pushes the SeekToGps goal and also the point to be seeked.
*    * If the waypoint is supposed to have a cone at the location:
*        * Push the DiscoverCone goal so that the cone detector for the robot can see the cone.
*        * Push the MoveToCone goal to touch the cone.
*        * Push the MoveFromCone to give room to manuever to the next waypoint.
*    * If there is another point in the list, repeat the process for the next point. Otherwise, the goal is achieved and the problem solver indicates success.
*/
class SolveRoboMagellan : public StrategyFn {
private:
	/*! \brief The possible states in the state machine. */
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
	static std::vector<GPS_POINT> gps_points_;			// Ordered list of waypoints to traverse.
	long int index_next_point_to_seek_;					// Index into list of GPS points to seek to next.
	victoria_perception::AnnotateDetectorImage annotator_request_;	// The annotation request.
	STATE state_;										// State of state machine.
	YAML::Node waypoints_;								// Waypoints in yaml format.
	
	// ROS node handle.
	ros::NodeHandle nh_;

	static long int g_count_Fix_msgs_received_;		// Count of how many Fix messages received.
	static sensor_msgs::NavSatFix g_first_Fix_msg_;	// Captured copy of first Fix message received.
	static sensor_msgs::NavSatFix g_last_Fix_msg_;	// Captured copy of last Fix message received.

	/*! \brief Process one Fix message.
	* \param msg The message via the ROS topic framework.
	*/
	static void fixCb(const sensor_msgs::NavSatFixConstPtr& msg);

  	/*! \brief Calculate heading between two GPS points in the GPS coordinate system--accurate 
  	* for distances for RoboMagellan.
	* \param from 	Beginning GPS point.
	* \param to 	Ending GPS point.
	* Return the heading between the 'from' and 'to' points.
  	*/
	static double headingInGpsCoordinates(sensor_msgs::NavSatFix from, sensor_msgs::NavSatFix to) {
	    double lat1 = angles::from_degrees(from.latitude);
	    double lon1 = angles::from_degrees(from.longitude);
	    double lat2 = angles::from_degrees(to.latitude);
	    double lon2 = angles::from_degrees(to.longitude);
	    double dLon = lon2 - lon1;
	 
	    double y = sin(dLon) * cos(lat2);
	    double x = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dLon));
	 
	    return atan2(y, x);
	}

	/*! \brief Parse the given yaml file and create an internal list equivalent of GPS points.
	* \param waypoint_yaml_path 	The path of the yaml file to be parsed.
	*/
	static void createGpsPointSet(std::string waypoint_yaml_path);
	 
	 
 	/*! \brief Calculate distance between two GPS points. Accurate for distances used in RoboMagellan.
	* See the 'haversine' formula in http://www.movable-type.co.uk/scripts/latlong.html
	* \param from 	Beginning GPS point.
	* \param to 	Ending GPS point.
	* Return the great circle distance in meters between the 'from' and 'to' points.
	*/
 	static double greatCircleDistance(sensor_msgs::NavSatFix from, sensor_msgs::NavSatFix to) {
	    double lat1 = angles::from_degrees(from.latitude);
	    double lon1 = angles::from_degrees(from.longitude);
	    double lat2 = angles::from_degrees(to.latitude);
	    double lon2 = angles::from_degrees(to.longitude);
	    double dLat = lat2 - lat1;
	    double dLon = lon2 - lon1;
	 
	    double a = (sin(dLat / 2.0) * sin(dLat / 2.0)) + 
	               ((cos(lat1) * cos(lat2)) *
	                (sin(dLon / 2.0) * sin(dLon / 2.0)));
	    double c = 2.0 * atan2(sqrt(a), sqrt(1 - a));
	    
	    return 6371000.0 * c;
	}

	/*! \brief Reset goal. After this, someone must request the goal again and it will start over. */
	void resetGoal();

	/*! \brief Move close to a GPS waypoint. */
	RESULT_T doMoveToGpsPoint(std::ostringstream& out_ss);

	/*! \brief Rotate until the cone is found in the video stream. */
	RESULT_T doFindConeInCamera(std::ostringstream& out_ss);

	/*! \brief Move towards and touch a cone. */
	RESULT_T doMoveToCone(std::ostringstream& out_ss);

	/*! \brief Move away from the cone in order to be ready to move to any next point. */
	RESULT_T doMoveFromCone(std::ostringstream& out_ss);

	/*! \brief Advance to the next waypoint in the list. */
	RESULT_T doAdvanceToNextPoint(std::ostringstream& out_ss);

	// Singleton pattern.
	SolveRoboMagellan();
	SolveRoboMagellan(SolveRoboMagellan const&) {}
	SolveRoboMagellan& operator=(SolveRoboMagellan const&) {}

public:
	RESULT_T tick();

	/*! \brief Return the list of GPS points that define the problem to be solved. */
	const std::vector<GPS_POINT>* getGpsPoints() { return &gps_points_; }

	/*! \brief Return the name of the problem solver. */
	const std::string& goalName() {
		static const std::string goal_name = "SolveRoboMagellan";
		return goal_name;
	}

	/*! \brief Return the class name. */
	const std::string& name() { 
		static const std::string name = "SolveRoboMagellan";
		return name;
	};

	static StrategyFn& singleton();

	/*! brief Convert the state enumeration value into a string.
	* \param state 	The state enumeration value.
	* Returns the string equivalent of the state value.
	*/
	const std::string& stateName(STATE state) {
		static const std::string setup = "SETUP";
		static const std::string move_to_gps_point = "MOVE_TO_GPS_POINT";
		static const std::string find_cone_in_camera = "FIND_CONE_IN_CAMERA";
		static const std::string move_to_cone = "MOVE_TO_CONE";
		static const std::string move_from_cone = "MOVE_FROM_CONE";
		static const std::string advance_to_next_point = "ADVANCE_TO_NEXT_POINT";
		static const std::string unknown = "!!UNKNOWN!!";

		switch (state) {
			case SETUP:					return setup;
			case MOVE_TO_GPS_POINT:		return move_to_gps_point;
			case FIND_CONE_IN_CAMERA:	return find_cone_in_camera;
			case MOVE_TO_CONE:			return move_to_cone;
			case MOVE_FROM_CONE:		return move_from_cone;
			case ADVANCE_TO_NEXT_POINT:	return advance_to_next_point;
			default:					return unknown;
		}
	}

};

#endif // __VICTORIA_NAVIGATION_SOLVE_ROBOMAGELLAN