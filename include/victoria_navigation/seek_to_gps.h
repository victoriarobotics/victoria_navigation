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

class SeekToGps : public StrategyFn {
private:
	typedef enum {
		kSEEKING_POINT,
		kSETUP,
		kROTATING_TO_DISCOVER
	} STATE;

	// Parameters.
	string cmd_vel_topic_name_;			// Topic name containing cmd_vel message.
	string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	string fix_topic_name_;				// Topic name containing fix message.
	string imu_topic_name_;				// Topic name containing IMU message. Used only if use_imu_ => true.
	double magnetic_declination_;		// Magnetic declination adjustment to be applied to IMU.
	string odometry_topic_name_;		// Topic name containing Odometry message.
	bool use_imu_;						// True => use IMU instead of Odometry as true robot heading.
	string waypoint_yaml_path_;			// Path to yaml file containing waypoints.

	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber fix_sub_;
	ros::Subscriber imu_sub_;			// Used only if use_imu => true.
	ros::Subscriber odometry_sub_;

	// Algorithm variables.
	double goal_yaw_degrees_;					// Goal heading.
	long int index_next_point_to_seek_;			// Index into list of GPS points to seek to next.
	bool odometry_capturered_;					// Odometry message has been captured.
	geometry_msgs::Quaternion previous_pose_;	// Pose from last Odometry message.
	nav_msgs::Odometry starting_Odometry_msg_;	// Odometry mesage at start of rotation strategy.
	double starting_yaw_;						// Starting yaw.
	double total_rotated_yaw_;					// Integration of rotational yaw since start.
	STATE state_;
	YAML::Node waypoints_;						// Waypoints in yaml format.
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

	// Reset goal. After this, someone must request the goal again and it will start over.
	void resetGoal();

	// Normalize an Euler angle into [0..360].
	double normalizeEuler(double yaw);

	static double degrees(double x) { return x * 180.0 / M_PI; }
	static double radians(double x) { return x * M_PI / 180.0; }
 
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
			case kROTATING_TO_DISCOVER:				return "kROTATING_TO_DISCOVER";
			default:								return "!!UNKNOWN!!";
		}
	}

};

#endif