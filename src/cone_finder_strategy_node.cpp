#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib_msgs/GoalStatus.h>

#include "victoria_navigation/move_to_cone.h"
#include "victoria_navigation/strategy_context.h"
#include "victoria_navigation/strategy_exception.h"
#include <stdint.h>

int debug_last_message = 0; // So that we only emit messages when things change.
		
void logIfChanged(int id, const char* message) {
	if (id != debug_last_message) {
		ROS_INFO_STREAM(message);
		debug_last_message = id;
	}	
}

vector<StrategyFn*> behaviors;

uint8_t resultToStatus(StrategyFn::RESULT_T result) {
	switch (result) {
		case StrategyFn::UNUSED_START:
			return actionlib_msgs::GoalStatus::LOST;
			
		case StrategyFn::FAILED:
			return actionlib_msgs::GoalStatus::ABORTED;

		case StrategyFn::FATAL:
			return actionlib_msgs::GoalStatus::LOST;

		case StrategyFn::RESTART_LOOP:
			return actionlib_msgs::GoalStatus::ACTIVE;

		case StrategyFn::RUNNING:
			return actionlib_msgs::GoalStatus::PENDING;

		case StrategyFn::SUCCESS:
			return actionlib_msgs::GoalStatus::SUCCEEDED;

		case StrategyFn::UNUSED_END:
			return actionlib_msgs::GoalStatus::LOST;
	}

	return actionlib_msgs::GoalStatus::LOST;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cone_finder_strategy_node");
	StrategyContext& strategy_context = StrategyContext::singleton();

	// ROS node handle.
	ros::NodeHandle nh("~");

	ros::Publisher strategyStatusPublisher = nh.advertise<actionlib_msgs::GoalStatus>("/cone_strategy", 1);

	ros::Rate rate(10); // Loop rate

	behaviors.push_back(&MoveToCone::singleton());

	actionlib_msgs::GoalStatus goalStatus;

	strategy_context.blackboard["needToMoveToCone"] = "T";

	while (ros::ok()) {
		try { // Emplement Sequence behavior
			rate.sleep();
			ros::spinOnce();

			//ROS_INFO_STREAM("--- ---- ---- ---- Begin of strategy loop ---- ---- ---- ----");
			for(vector<StrategyFn*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
				StrategyFn::RESULT_T result = ((*it)->tick)();
				goalStatus.goal_id.stamp = ros::Time::now();
				goalStatus.goal_id.id = "cone_finder_strategy_node";
				goalStatus.status = resultToStatus(result);
				goalStatus.text = "[strategy_node] executed: " + ((*it)->name());
				strategyStatusPublisher.publish(goalStatus);

				if (result == StrategyFn::RESTART_LOOP) {
					break; // throw new StrategyException("RESTART_LOOP");
				}

				if (result == StrategyFn::FATAL) {
					ROS_INFO_STREAM("[_strategy_node] function: " << ((*it)->name()) << ", FATAL result, exiting");
					return -1;
				}

				if (result == StrategyFn::RUNNING) {
					break; // throw new StrategyException("RESTART_LOOP");
				}

				if (result == StrategyFn::SUCCESS) {
					continue;
				}

				if (result == StrategyFn::FAILED) {
					break;
				}
			}
		} catch(StrategyException* e) {
			//ROS_INFO_STREAM("[_strategy_node] StrategyException: " << e->what());
			// Do nothing.
		}
	}

	return 0;
}

