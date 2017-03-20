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

#ifndef __MOVE_TO_CONE
#define __MOVE_TO_CONE

#include <ros/ros.h>
#include <string>

#include "victoria_perception/ObjectDetector.h"
#include "victoria_navigation/strategy_fn.h"

class MoveToCone : public StrategyFn {
private:
	typedef enum {
		kMOVING_TO_CENTERING_POSITION,
		kMOVING_TO_TOUCH
	} STATE;

	// ROS node handle.
	ros::NodeHandle nh_;

	// Parameters.
	string cmd_vel_topic_name_;			// Topic name containing cmd_vel message.
	string cone_detector_topic_name_;	// Topic name containing cone_detector message

	// Publishers.
	ros::Publisher cmd_vel_pub_;
	ros::Publisher current_strategy_pub_;
	ros::Publisher strategy_status_publisher_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;

	// Algorithm variables.
	string last_reported_strategy_;
	STATE state_;
	
	// Process one cone detector topic message.
	long int object_detections_received_;
	victoria_perception::ObjectDetector last_object_detected_;
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	// Reset goal. After this, someone must request the goal again and it will start over.
	void resetGoal();

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

	// Singleton pattern.
	MoveToCone();
	MoveToCone(MoveToCone const&) {};
	MoveToCone& operator=(MoveToCone const&) {};

public:
	RESULT_T tick();

	string goalRequestParam() { return "/strategy/need_to_move_cone"; }

	string name() { return string("MoveToCone"); };

	static MoveToCone& singleton();

	string stateName(STATE state) {
		switch (state) {
			case kMOVING_TO_CENTERING_POSITION:		return "kMOVING_TO_CENTERING_POSITION";
			case kMOVING_TO_TOUCH:					return "kMOVING_TO_TOUCH";
			default:								return "!!UNKNOWN!!";
		}
	}

};

#endif