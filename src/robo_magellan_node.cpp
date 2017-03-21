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

#include <cassert>
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

StrategyFn::RESULT_T doStrategy(vector<StrategyFn*>& behaviors, ros::Publisher& strategyStatusPublisher) {
    actionlib_msgs::GoalStatus goalStatus;
    StrategyFn::RESULT_T result = StrategyFn::FATAL;
    ros::Rate rate(10); // Loop rate

    while (ros::ok()) {
        try {
            rate.sleep();
            ros::spinOnce();

            for (vector<StrategyFn*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
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
                    return result;
                }

                if (result == StrategyFn::RUNNING) {
                    continue;
                }

                if (result == StrategyFn::SUCCESS) {
                    return result;
                }

                if (result == StrategyFn::FAILED) {
                    continue;
                }
            }
        } catch (StrategyException* e) {
            //ROS_INFO_STREAM("[_strategy_node] StrategyException: " << e->what());
            // Do nothing.
        }
    }

    return result;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robo_magellan_node");

    // ROS node handle.
    typedef enum {
        kDISCOVER_CONE,
        kMOVE_TO_CONE
    } GOAL;

    GOAL current_goal;
    bool do_debug_strategy;
    StrategyFn::RESULT_T result;

    ros::NodeHandle nh("~");

	assert(ros::param::get("~do_debug_strategy", do_debug_strategy));
    ros::Publisher strategyStatusPublisher = nh.advertise<actionlib_msgs::GoalStatus>("/cone_strategy", 1);

    ros::Rate rate(10); // Loop rate

    DiscoverCone& discover_cone = DiscoverCone::singleton();
    MoveToCone& move_to_cone = MoveToCone::singleton();
	ROS_INFO_COND(do_debug_strategy, "[robo_magellan_node] New goal: DiscoverCone");

    behaviors.push_back(&DiscoverCone::singleton());
    behaviors.push_back(&MoveToCone::singleton());

    // Set initial goal
    current_goal = kDISCOVER_CONE;
    ros::param::set(discover_cone.goalRequestParam(), true);
    while (ros::ok()) {
        result = doStrategy(behaviors, strategyStatusPublisher);
        if (result == StrategyFn::SUCCESS) {
            switch (current_goal) {
            case kDISCOVER_CONE:
            	current_goal = kMOVE_TO_CONE;
            	ros::param::set(move_to_cone.goalRequestParam(), true);
            	ROS_INFO_COND(do_debug_strategy, "[robo_magellan_node] New goal: MoveToCone");
                break;

            case kMOVE_TO_CONE:
	            ROS_INFO("YAY SUCCESS");
	            return 0;

            default:
                ROS_ERROR("[robo_magellan_node] !!! INVALID CURRENT GOAL");
                return -1;
            }

        } else if (result == StrategyFn::FAILED) {
            continue;
        } else {
            ROS_INFO("BOO FAILURE");
            return -1;
        }
    }

    return 0;
}

