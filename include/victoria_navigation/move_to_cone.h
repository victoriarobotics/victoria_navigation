#ifndef __MOVE_TO_CONE
#define __MOVE_TO_CONE

#include <ros/ros.h>
#include <string>

#include "victoria_perception/ObjectDetector.h"
#include "victoria_navigation/strategy_context.h"
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
	StrategyContext& strategy_context_ = StrategyContext::singleton();

	// Process one cone detector topic message.
	long int object_detections_received_;
	victoria_perception::ObjectDetector last_object_detected_;
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	// Publish current strategy (if changed).
	void publishCurrentStragety(string strategy);

	// Singleton pattern.
	MoveToCone();
	MoveToCone(MoveToCone const&) : strategy_context_(StrategyContext::singleton()) {};
	MoveToCone& operator=(MoveToCone const&) {};

public:
	RESULT_T tick();

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