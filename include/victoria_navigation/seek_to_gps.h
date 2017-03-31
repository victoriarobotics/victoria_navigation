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

#ifndef __SEEK_TO_GPS
#define __SEEK_TO_GPS

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "victoria_perception/ObjectDetector.h"
#include "victoria_navigation/strategy_fn.h"

// A behavior that attempts to get close to a GPS waypoint.
//
// The goal waypoint is at the top of the g_point_stack_.
//
// The behavior will attemptm to get "close enough" to the waypoint. By "close enough" is meant:
//		* If the waypoint DOES NOT have a cone, try to get within the parameter "gps_close_distance_meters"
//		  of the target GPS location.
//		* If the waypoint DOES have a cone, also try to get within "gps_close_distance_meters" but
//		  if a cone is detected before that, stop where it is detected.
//
// The behavior depends on a few other components and parameters..
//		* "cone_detector_topic_name" is a topic listened to for an indication if a RoboMagellan cone is detected.
//		* "cmd_vel_topic_name" defines a topic to be used for moving the robot. Messages will be published
//		  to that topic. The robot will end up in a stopped state at the end of this behavior.
//		* "fix_topic_name" defines a topic lined to for the current GPS of the robot. It should be published
//		  frequently enough so that the robot won't veer dangerously close to the target before it gets
//		  another GPS fix report.
//		* "gps_close_distance_meters" defines the distance tolerance for being "close enough" to a waypoint.
//		* "imu_topic_name" defines a topic that MIGHT be listened to in order to determine the current
//		  heading. This is controlled by the "use_imu" parameter. If it is true, the IMU will be consulted.
//		  If it is false, the Odometry will be consulted.
//		* The Imu messages are assumed to be relative to magnetic north. This parameter gives the
//		  correction value to convert the orientation to true north, as used by the GPS.
//		* "odometry_topic_name" defines a topic the MIGHT be listened to in order to determine the current
//		  heading. See the discussion of "imu_topic_name" above.
//		* "use_imu" -- See the discussion of "imu_topic_name" above.
//
// The behavior works as follows:
//	* Wait until messages are received from the cone detector, Odometry, GPS and IMU.
//	* If the cone is seen, indicate SUCCESS and stop the robot.
//  * The first time the behavior is attempted on the waypoint, setup to rotate towards the goal
//	  waypoint. This orientation will happen again whenever the robot seems to have veered too
//	  far from a straight line course. Having rotated, attempt to move in a straight line to the
//	  waypoint.
//	* If the robot is "close enough" to the goal waypoint, indicate SUCCESS and stop the robot.
//	* Otherwise, keep moving towards the waypoint. If the robot heading veers too far from the
//	  waypoint, perform another rotation.
//
// POSSIBLE IMPROVEMENTS:
//	* Compute a sense of heading from the GPS. ROS doesn't publish the GPS heading, and the
//	  NMEA messages that show heading certainly aren't always timely or even vaguely accurate
//	  for different GPS sensors and the expected rate of travel of the robot. Given how
//	  inaccurate the GPS Fix message results are, this may not be particularly useful.
//	* Perform some sort of fusion between Odometry and Imu for a better sense of heading.
//	* Have the MoveToCone behavior update the Odometry sense of position, by assuming that
//	  the Odometry has drifted over time and the GPS Fix is a more accurate indicator of
//	  pose.
//	* Detect stale GPS, Odometry and Imu data and deal with it.
// 	* Only stop when a RoboMagellan cone is detected if that cone is acceptable. Viz., it's the
//	  appropriate size for the distance and is vaguely in the right direction.

class SeekToGps : public StrategyFn {
private:
	typedef enum {
		kSEEKING_POINT,			// Move robot towards the current GPS point.
		kSETUP,					// Gather initial state before attempting to get close to the GPS point.
		kROTATING_TO_HEADING
	} STATE;

	// Parameters.
	string cmd_vel_topic_name_;			// Topic name containing cmd_vel message.
	string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	string fix_topic_name_;				// Topic name containing fix message.
	double gps_close_distance_meters_;	// How close to seek the point using GPS before assuming it's "close enough".
	string imu_topic_name_;				// Topic name containing IMU message. Used only if use_imu_ => true.
	double magnetic_declination_;		// Magnetic declination adjustment to be applied to IMU.
	string odometry_topic_name_;		// Topic name containing Odometry message.
	bool solve_using_odom_;				// Solve using Odomentry only, no GPS or IMU.
	bool use_imu_;						// True => use IMU instead of Odometry as true robot heading.

	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber fix_sub_;
	ros::Subscriber imu_sub_;			// Used only if use_imu => true.
	ros::Subscriber odometry_sub_;

	// Algorithm variables.
	double goal_yaw_degrees_;					// Goal heading.
	geometry_msgs::Quaternion previous_pose_;	// Pose from last Odometry message.
	nav_msgs::Odometry starting_Odometry_msg_;	// Odometry mesage at start of rotation strategy.
	double starting_yaw_;						// Starting yaw.
	double total_rotated_yaw_;					// Integration of rotational yaw since start.
	STATE state_;								// State of state machine.
	double goal_yaw_degrees_delta_threshold_;	// If delta is less than this, don't bother rotating.
	
	// Process one ConeDetector topic message.
	long int count_ObjectDetector_msgs_received_;
	victoria_perception::ObjectDetector last_ObjectDetector_msg_;
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	// Process one Fix topic message.
	long int count_Fix_msgs_received_;
	sensor_msgs::NavSatFix last_Fix_msg_;
	void fixCb(const sensor_msgs::NavSatFixConstPtr& msg);

	// Process one Imu message.
	long int count_Imu_msgs_received_;
	sensor_msgs::Imu last_Imu_msg_;
	void imuCb(const sensor_msgs::ImuConstPtr& msg);
	
	// Process one Odometry topic message.
	long int count_Odometry_msgs_received_;
	nav_msgs::Odometry last_Odometry_msg_;
	void odometryCb(const nav_msgs::OdometryConstPtr& msg);

	// Reset global state so this behavior can be used to solve the next problem.
	void resetGoal();

	// Normalize an Euler angle into [0..360).
	double normalizeEuler(double yaw);

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
 	
 	double odomBearing(double x1, double y1, double x2, double y2);
	 
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

	// Singleton pattern.
	SeekToGps();
	SeekToGps(SeekToGps const&) {};
	SeekToGps& operator=(SeekToGps const&) {};

public:
	RESULT_T tick();

	string goalName() { return "/strategy/seek_to_gps_point"; }

	string name() { return string("SeekToGps"); };

	static SeekToGps& singleton();

	string stateName(STATE state) {
		switch (state) {
			case kSEEKING_POINT:					return "kSEEKING_POINT";
			case kSETUP:							return "kSETUP";
			case kROTATING_TO_HEADING:				return "kROTATING_TO_HEADING";
			default:								return "!!UNKNOWN!!";
		}
	}

};

#endif