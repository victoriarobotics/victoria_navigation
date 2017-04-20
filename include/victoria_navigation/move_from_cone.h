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

#ifndef __VICTORIA_NAVIGATION_MOVE_FROM_CONE
#define __VICTORIA_NAVIGATION_MOVE_FROM_CONE

#include <ros/ros.h>
#include <string>

#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"
#include "victoria_perception/ObjectDetector.h"

// A behavior that attempts to move from a RoboMagellan cone in order to possibly move to another cone..
//

class MoveFromCone : public StrategyFn {
private:
	enum STATE {
		MOVE_START,					// Start of goal
		MOVING_AWAY					// Moving away.
	};

	// Parameters.
	std::string cmd_vel_topic_name_;	// Topic name containing cmd_vel message.
\
	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Algorithm variables.
	STATE state_;
	ros::Time move_start_time_;
	ros::ServiceClient coneDetectorAnnotatorService_;	// For annotating the cone detector image.
	victoria_perception::AnnotateDetectorImage annotator_request_;	// The annotation request.
	
	// Reset goal. After this, someone must request the goal again and it will start over.
	void resetGoal();

	// Singleton pattern.
	MoveFromCone();
	MoveFromCone(MoveFromCone const&) {}
	MoveFromCone& operator=(MoveFromCone const&) {}

public:
	RESULT_T tick();

	const std::string& goalName() {
		static const std::string goal_name = "MoveFromCone";
		return goal_name;
	}

	const std::string& name() {
		static const std::string name = "MoveFromCone";
		return name;
	}

	static StrategyFn& singleton();

	const std::string& stateName(STATE state) {
		static const std::string start = "START";
		static const std::string moving_away = "MOVING_AWAY";
		static const std::string unknown = "!!UNKNOWN!!";

		switch (state) {
			case MOVE_START:		return start;
			case MOVING_AWAY:		return moving_away;
			default:				return unknown;
		}
	}

};

#endif  // __VICTORIA_NAVIGATION_MOVE_FROM_CONE