// Copyright <YEAR> <COPYRIGHT HOLDER>

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

#ifndef __DISCOVER_CONE
#define __DISCOVER_CONE

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "victoria_perception/ObjectDetector.h"
#include "victoria_navigation/strategy_fn.h"

class DiscoverCone : public StrategyFn {
private:
	typedef enum {
		kCAPTURE_ODOMETRY,
		kROTATING_TO_DISCOVER
	} STATE;

	// ROS node handle.
	ros::NodeHandle nh_;

	// Parameters.
	string cmd_vel_topic_name_;			// Topic name containing cmd_vel message.
	string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	string odometry_topic_name_;		// Topic name containing Odometry message.

	// Publishers.
	ros::Publisher cmd_vel_pub_;
	ros::Publisher current_strategy_pub_;
	ros::Publisher strategy_status_publisher_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber odometry_sub_;

	// Algorithm variables.
	string last_reported_strategy_;
	bool odometry_capturered_;					// Odometry message has been captured.
	geometry_msgs::Quaternion previous_pose_;	// Pose from last Odometry message.
	nav_msgs::Odometry starting_Odometry_msg_;	// Odometry mesage at start of rotation strategy.
	double starting_yaw_;						// Starting yaw.
	double total_rotated_yaw_;					// Integration of rotational yaw since start.
	STATE state_;
	
	// Process one ConeDetector topic message.
	long int count_ObjectDetector_msgs_received_;
	victoria_perception::ObjectDetector last_ObjectDetector_msg_;
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	// Reset goal. After this, someone must request the goal again and it will start over.
	void resetGoal();

	// Normalize an Euler angle into [0..360].
	double normalizeEuler(double yaw);

	// Process one Odometry topic message;
	long int count_Odometry_msgs_received_;
	nav_msgs::Odometry last_Odometry_msg_;
	void odometryCb(const nav_msgs::OdometryConstPtr& msg);

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

	// Singleton pattern.
	DiscoverCone();
	DiscoverCone(DiscoverCone const&) {};
	DiscoverCone& operator=(DiscoverCone const&) {};

public:
	RESULT_T tick();

	string goalRequestParam() { return "/strategy/need_to_discover_cone"; }

	string name() { return string("DiscoverCone"); };

	static DiscoverCone& singleton();

	string stateName(STATE state) {
		switch (state) {
			case kCAPTURE_ODOMETRY:					return "kCAPTURE_ODOMETRY";
			case kROTATING_TO_DISCOVER:				return "kROTATING_TO_DISCOVER";
			default:								return "!!UNKNOWN!!";
		}
	}

};

#endif