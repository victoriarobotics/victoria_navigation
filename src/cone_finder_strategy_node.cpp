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

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib_msgs/GoalStatus.h>

#include "victoria_navigation/discover_cone.h"
#include "victoria_navigation/move_to_cone.h"
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

	// ROS node handle.
	ros::NodeHandle nh("~");

	ros::Publisher strategyStatusPublisher = nh.advertise<actionlib_msgs::GoalStatus>("/cone_strategy", 1);

	ros::Rate rate(10); // Loop rate

	behaviors.push_back(&DiscoverCone::singleton());
	behaviors.push_back(&MoveToCone::singleton());

	actionlib_msgs::GoalStatus goalStatus;

	//###	ros::param::set("/strategy/need_to_move_cone", true);
	ros::param::set("/strategy/need_to_discover_cone", true);

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

