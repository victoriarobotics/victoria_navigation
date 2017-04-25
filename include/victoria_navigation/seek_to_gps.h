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

#ifndef __VICTORIA_NAVIGATION_SEEK_TO_GPS
#define __VICTORIA_NAVIGATION_SEEK_TO_GPS

#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"
#include "victoria_perception/ObjectDetector.h"

//* A SeekToGps class.
/**
* A problem solver that attempts to get close to a GPS waypoint.
*
* The goal waypoint is at the top of the g_point_stack_.
*
* The problem solver will attemptm to get "close enough" to the waypoint. By "close enough" is meant:
*		* If the waypoint DOES NOT have a cone, try to get within the parameter "gps_close_distance_meters"
*		  of the target GPS location.
*		* If the waypoint DOES have a cone, also try to get within "gps_close_distance_meters" but
*		  if a cone is detected before that, stop where it is detected.
*
* The problem solver depends on a few other components and parameters..
*		* "cone_detector_topic_name" is a topic listened to for an indication if a RoboMagellan cone is detected.
*		* "cmd_vel_topic_name" defines a topic to be used for moving the robot. Messages will be published
*		  to that topic. The robot will end up in a stopped state at the end of this problem solver.
*		* "fix_topic_name" defines a topic lined to for the current GPS of the robot. It should be published
*		  frequently enough so that the robot won't veer dangerously close to the target before it gets
*		  another GPS fix report.
*		* "gps_close_distance_meters" defines the distance tolerance for being "close enough" to a waypoint.
*		* "imu_topic_name" defines a topic that MIGHT be listened to in order to determine the current
*		  heading. This is controlled by the "use_imu" parameter. If it is true, the IMU will be consulted.
*		  If it is false, the Odometry will be consulted.
*		* "magnetic_declination" The Imu messages are assumed to be relative to magnetic north. 
		  This parameter gives the correction value to convert the orientation to true north, as used by the GPS.
*		* "odometry_topic_name" defines a topic the MIGHT be listened to in order to determine the current
*		  heading. See the discussion of "imu_topic_name" above.
*		* "use_imu" -- See the discussion of "imu_topic_name" above.
*
* The problem solver works as follows:
*	* Wait until messages are received from the cone detector, Odometry, GPS and IMU.
*	* If the cone is seen, indicate SUCCESS and stop the robot.
*	* If the robot is "close enough" to the goal waypoint, indicate SUCCESS and stop the robot.
*	* Otherwise, keep moving towards the waypoint. The robot continually corrects the heading 
*	  in an attempt to smoothly head directly to the waypoint.
*
* POSSIBLE IMPROVEMENTS:
*	* Detect stale GPS, Odometry and Imu data and deal with it.
* 	* Only stop when a RoboMagellan cone is detected if that cone is acceptable. Viz., it's the
*	  appropriate size for the distance and is vaguely in the right direction.
*/

class SeekToGps : public StrategyFn {
private:
	/*! \brief The possible states in the state machine. */
	enum STATE {
		SEEKING_POINT,			// Move robot towards the current GPS point.
		SETUP					// Gather initial state before attempting to get close to the GPS point.
	};

	/*! \brief Holds an interest collection of information about the current robot position
	 * and the goal waypoint. */
	typedef struct WAYPOINT_STATS {
		double heading_to_waypoint_delta;	// Heading error from current position to waypoint.
		double robot_to_waypoint_distance;	// Distance from current position to waypoint.
		double robot_to_waypoint_heading;	// Heading from current position to waypoint.
		double robot_true_heading;			// True (not magnetic) heading of robot.
	} WAYPOINT_STATS;

	// Parameters.
	std::string cmd_vel_topic_name_;		// Topic name containing cmd_vel message.
	std::string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	std::string fix_topic_name_;			// Topic name containing fix message.
	double gps_close_distance_meters_;		// How close to seek the point using GPS before assuming it's "close enough".
	float ignore_cone_until_within_meters_; // Ignore the cone detector until within this distance of the goal point.
	std::string imu_topic_name_;			// Topic name containing IMU message. Used only if use_imu_ => true.
	float linear_move_meters_per_sec_;		// Rate to move forward (meters/sec).
	double magnetic_declination_;			// Magnetic declination adjustment to be applied to IMU.
	std::string odometry_topic_name_;		// Topic name containing Odometry message.
	bool solve_using_odom_;					// Solve using Odomentry only, no GPS or IMU.
	bool use_imu_;							// True => use IMU instead of Odometry as true robot heading.
	float yaw_turn_radians_per_sec_;		// Rate to turn around z azis (radians/sec)

	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber fix_sub_;
	ros::Subscriber imu_sub_;			// Used only if use_imu => true.
	ros::Subscriber odometry_sub_;

	// Algorithm variables.
	geometry_msgs::Quaternion previous_pose_;		// Pose from last Odometry message.
	nav_msgs::Odometry starting_odometry_msg_;		// Odometry mesage at start of rotation strategy.
	double starting_yaw_;							// Starting yaw.
	double total_rotated_yaw_;						// Integration of rotational yaw since start.
	STATE state_;									// State of state machine.
	double heading_to_waypoint_degrees_delta_threshold_;	// If delta is less than this, don't bother rotating.
	ros::ServiceClient coneDetectorAnnotatorService_;	// For annotating the cone detector image.
	victoria_perception::AnnotateDetectorImage annotator_request_;	// The annotation request.
	
	// Process one ConeDetector topic message.
	long int count_object_detector_msgs_received_;
	victoria_perception::ObjectDetector last_object_detector_msg_;
	/*! \brief Capture the last cone detector topic message. */
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	// Process one Fix topic message.
	long int count_fix_msgs_received_;
	sensor_msgs::NavSatFix last_fix_msg_;
	/*! \brief Capture the last GPS Fix message. */
	void fixCb(const sensor_msgs::NavSatFixConstPtr& msg);

	// Process one Imu message.
	long int count_imu_msgs_received_;
	sensor_msgs::Imu last_imu_msg_;
	/*! \brief Capture the last IMU message. */
	void imuCb(const sensor_msgs::ImuConstPtr& msg);
	
	// Process one Odometry topic message.
	long int count_odometry_msgs_received_;
	nav_msgs::Odometry last_odometry_msg_;
	/*! \brief capture the last Odom message. */
	void odometryCb(const nav_msgs::OdometryConstPtr& msg);

	/*! \brief Reset global state so this problem solver can be used to solve the next problem. */
	void resetGoal();

 	/*! \brief Calculate heading between two GPS points in the GPS coordinate system--
 	 * accurate for distances for RoboMagellan. */
	double headingInGpsCoordinates(sensor_msgs::NavSatFix from, sensor_msgs::NavSatFix to) {
	    double lat1 = angles::from_degrees(from.latitude);
	    double lon1 = angles::from_degrees(from.longitude);
	    double lat2 = angles::from_degrees(to.latitude);
	    double lon2 = angles::from_degrees(to.longitude);
	    double dLon = lon2 - lon1;
	 
	    double y = sin(dLon) * cos(lat2);
	    double x = cos(lat1) * sin(lat2) - (sin(lat1) * cos(lat2) * cos(dLon));
	 
	    return atan2(y, x);
	}
 	
 	/*! \brief Calculate heading between two cartesian points. */
 	double odomHeading(double x1, double y1, double x2, double y2);
	 
 	/*! \brief Calculate distance between two GPS points. Accurate for distances used in RoboMagellan. */
	double greatCircleDistance(sensor_msgs::NavSatFix from, sensor_msgs::NavSatFix to) {
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

	/*! \brief Compute interesting statistics related to the current position and the goal waypoint. */
	WAYPOINT_STATS computeWaypointStats();

	// State handlers.
	/*! \brief Have the preconditions been met? */
	bool checkPreconditions();

	/*! \brief Setup any state before starting out. */
	RESULT_T doSetup(std::ostringstream& out_ss, WAYPOINT_STATS& waypoint_stats);

	/*! \brief Seek to the GPS waypoint. */
	RESULT_T doSeekingPoint(std::ostringstream& out_ss, WAYPOINT_STATS& waypoint_stats);
	
	// Singleton pattern.
	SeekToGps();
	SeekToGps(SeekToGps const&) {}
	SeekToGps& operator=(SeekToGps const&) {}

public:
	RESULT_T tick();

	const std::string& goalName() {
		static const std::string goal_name = "SeekToGps";
		return goal_name;
	}

	const std::string& name() {
		static const std::string name = "SeekToGps";
		return name;
	}

	static StrategyFn& singleton();

	/*! \brief Convert a state value to a string. */
	const std::string& stateName(STATE state) {
		static const std::string seeking_point = "SEEKING_POINT";
		static const std::string setup = "SETUP";
		static const std::string unknown = "!!UNKNOWN!!";

		switch (state) {
			case SEEKING_POINT:					return seeking_point;
			case SETUP:							return setup;
			default:								return unknown;
		}
	}

};

#endif // __VICTORIA_NAVIGATION_SEEK_TO_GPS