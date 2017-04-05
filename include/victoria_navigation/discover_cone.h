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

#ifndef __VICTORIA_NAVIGATION_DISCOVER_CONE
#define __VICTORIA_NAVIGATION_DISCOVER_CONE

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "victoria_perception/ObjectDetector.h"
#include "victoria_navigation/strategy_fn.h"

// A behavior that attempts to discover a RoboMagellan cone in the camera.
//
// The behavior depends on a few other components and parameters..
//		* "cone_detector_topic_name" is a topic listened to for an indication if a RoboMagellan cone is detected.
//		* "cmd_vel_topic_name" defines a topic to be used for moving the robot. Messages will be published
//		  to that topic. The robot will end up in a stopped state at the end of this behavior.
//		* "odometry_topic_name" defines a topic the MIGHT be listened to in order to determine the current
//		  heading. See the discussion of "imu_topic_name" above.
//
// The behavior works as follows:
//	* Wait until messages are received from the cone detector and Odometry.
//  * If the cone is seen, indicate SUCCESS and stop the robot.
//	* The first time this behavior is attempted, capture the current Odometry. This will be used
//	  to detect when a complete revolution has been made.
//	* If the robot hasn't yet made a complete revolution, rotate a bit.
//
// POSSIBLE IMPROVEMENTS:
//	* Find all cones in a complete rotation and choose the best.
//	* Choose the cone that is closest to the expected heading.

class DiscoverCone : public StrategyFn {
private:
	enum STATE {
		kCAPTURE_ODOMETRY,		// Capture the current Odometry.
		kROTATING_TO_DISCOVER	// Rotate until a RoboMagellan cone is discovered.
	};

	// Parameters.
	std::string cmd_vel_topic_name_;			// Topic name containing cmd_vel message.
	std::string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	std::string odometry_topic_name_;		// Topic name containing Odometry message.

	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber odometry_sub_;

	// Algorithm variables.
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

	// Process one Odometry topic message;
	long int count_Odometry_msgs_received_;
	nav_msgs::Odometry last_Odometry_msg_;
	void odometryCb(const nav_msgs::OdometryConstPtr& msg);

	// Singleton pattern.
	DiscoverCone();
	DiscoverCone(DiscoverCone const&) {};
	DiscoverCone& operator=(DiscoverCone const&) {};

public:
	RESULT_T tick();

	std::string goalName() { return "/strategy/need_to_discover_cone"; }

	std::string name() { return std::string("DiscoverCone"); };

	static DiscoverCone& singleton();

	std::string stateName(STATE state) {
		switch (state) {
			case kCAPTURE_ODOMETRY:					return "kCAPTURE_ODOMETRY";
			case kROTATING_TO_DISCOVER:				return "kROTATING_TO_DISCOVER";
			default:								return "!!UNKNOWN!!";
		}
	}

};

#endif // __VICTORIA_NAVIGATION_DISCOVER_CONE