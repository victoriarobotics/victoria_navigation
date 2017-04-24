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

#ifndef __VICTORIA_NAVIGATION_MOVE_TO_CONE
#define __VICTORIA_NAVIGATION_MOVE_TO_CONE

#include <ros/ros.h>
#include <string>

#include "victoria_sensor_msgs/DistanceDisplacement1D.h"
#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"
#include "victoria_perception/ObjectDetector.h"

//* A SeekToGps class.
/**
* A problem solver that attempts to move to a RoboMagellan cone.
*
* The problem solver depends on a few other components and parameters..
*		* "cone_detector_topic_name" is a topic listened to for an indication if a RoboMagellan cone is detected.
*		* "cmd_vel_topic_name" defines a topic to be used for moving the robot. Messages will be published
*		  to that topic. The robot will end up in a stopped state at the end of this problem solver.
*		* "distance_displacement_1d_topic_name" defines a topic to be used to detect a bumper hit.
*
* The problem solver works as follows:
*	* Wait until messages are received from the cone detector.
*   * If the cone is not seen for two seconds, indicate FAILURE and stop the robot.
*	* If a bumper hit is detected, either mechanically or optically, indicate SUCCESS and stop the robot.
*	* Otherwise, move the robot towards the cone.
*/

class MoveToCone : public StrategyFn {
private:
	/*! \brief The possible states in the state machine. */
	enum STATE {
		MOVE_START,						// Start of goal
		MOVING_TO_CENTERING_POSITION	// Rotate so the cone is more or less dead center ahead.
	};

	// Parameters.
	std::string cmd_vel_topic_name_;					// Topic name containing cmd_vel message.
	std::string cone_detector_topic_name_;				// Topic name containing cone_detector message.
	std::string distance_displacement_1d_topic_name_;	// Topic containing DistanceDisplacement1D message.
	float linear_move_meters_per_sec_;					// Rate to move forward (meters/sec).
	float yaw_turn_radians_per_sec_;					// Rate to turn around z azis (radians/sec)

	// Publishers.
	ros::Publisher cmd_vel_pub_;

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;
	ros::Subscriber distance_displacement_1d_topic_name_sub_;

	// Algorithm variables.
	bool bumper_hit_;					// Bumper hit detected.
	int cone_area_for_bumper_hit_;		// If equate_size_to_bumper_hit_, then a cone area >= this is considered a bumper_hit_.
	bool equate_size_to_bumper_hit_;	// True => if cone size >= cone_area_for_bumper_hit_ then it's equivalent to bumper_hit_.
	double field_of_view_degrees_;		// Camera field of view.
	STATE state_;						// State of the state machine.
	int sequential_detection_failures_;	// How many frames were seen without a cone detection?
	ros::Time time_last_saw_cone;		// Time since last saw that a cone was detected.
	ros::ServiceClient coneDetectorAnnotatorService_;	// For annotating the cone detector image.
	victoria_perception::AnnotateDetectorImage annotator_request_;	// The annotation request.
	
	// Process one cone detector topic message.
	long int count_ObjectDetector_msgs_received_;
	victoria_perception::ObjectDetector last_object_detected_;
	/*! \brief Process one ConeDetector message. */
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	/*! \brief Process DistanceDisplacement1D topic message. */
	void distanceDisplacement1DCb(const victoria_sensor_msgs::DistanceDisplacement1DConstPtr& msg);

	/*!  Reset goal. After this, someone must request the goal again and it will start over. */
	void resetGoal();

	// Singleton pattern.
	MoveToCone();
	MoveToCone(MoveToCone const&) {}
	MoveToCone& operator=(MoveToCone const&) {}

public:
	RESULT_T tick();

	const std::string& goalName() {
		static const std::string goal_name = "MoveToCone";
		return goal_name;
	}

	const std::string& name() {
		static const std::string name = "MoveToCone";
		return name;
	}

	static StrategyFn& singleton();

	/*! \brief Convert a state value to a string. */
	const std::string& stateName(STATE state) {
		static const std::string start = "START";
		static const std::string moving_to_centering_position = "MOVING_TO_CENTERING_POSITION";
		static const std::string unknown = "!!UNKNOWN!!";

		switch (state) {
			case MOVE_START:						return start;
			case MOVING_TO_CENTERING_POSITION:		return moving_to_centering_position;
			default:								return unknown;
		}
	}

};

#endif  // __VICTORIA_NAVIGATION_MOVE_TO_CONE