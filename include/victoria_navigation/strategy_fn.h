#ifndef __STRATEGY_FN
#define __STRATEGY_FN

#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

using namespace std;

class StrategyFn {
public:
	typedef struct GOAL_T {
		string 	goal_name;	// Name of goal.
		string 	goal_param;	// Optional parameter for goal.
		GOAL_T(string name, string param) : goal_name(name), goal_param(param) {}
	} GOAL_T;

	typedef struct GPS_POINT {
		double latitude;
		double longitude;
		bool has_cone;
		double x;			// Position relative to start.
		double y;			// Position relative to start.
		double bearing;		// Position bearing from start
		double distance;	// Distance to point.
	} GPS_POINT;

	typedef enum {
		UNUSED_START = 0,	// Do not use, must be first element.
		FAILED,				// Strategy failed, do not continue.
		FATAL,				// Something is fatally wrong.
		RESTART_LOOP,		// Strategy prempts downstream strategies, go to top of tree.
		RUNNING,			// Strategy is in progress.
		SUCCESS,			// Strategy succeeded, continue on.
		UNUSED_END			// Do not use, must be last element.
	} RESULT_T;

protected:
	static const char* RESULT_STR[];		// Map RESULT_T enum to string.
	static vector<GOAL_T> g_goal_stack_;	// Stack of goals to be solved.
	static vector<GPS_POINT> g_point_stack;	// Stack of goal GPS points, related to g_goal_stack.
	static const string g_empty_string;		// Singleton empty string.
	static RESULT_T g_last_goal_result_;	// Last result of current goal tick().

	// ROS node handle.
	ros::NodeHandle nh_;

	// Parameters.
	bool do_debug_strategy_;			// Emit info traces to help debug code.

	// Publishers.
	ros::Publisher current_strategy_pub_;
	ros::Publisher strategy_status_publisher_;

	string last_reported_strategy_;				// To prevent useless publication of same information.

	void publishCurrentStragety(string strategy) {
		std_msgs::String msg;
		msg.data = strategy;
		if (strategy != last_reported_strategy_) {
			last_reported_strategy_ = strategy;
			current_strategy_pub_.publish(msg);
			ROS_INFO_COND(do_debug_strategy_, "[StrategyFn::publishCurrentStragety] strategy: %s", strategy.c_str());
		}
	}

	void publishStrategyProgress(string strategy_name, string progress) {
		actionlib_msgs::GoalStatus 	goal_status;
		ostringstream ss;

		ss << "[" << strategy_name << "] " << progress;
		goal_status.goal_id.stamp = ros::Time::now();
		goal_status.goal_id.id = strategy_name;
		goal_status.status = actionlib_msgs::GoalStatus::ACTIVE;
		goal_status.text = ss.str();
		strategy_status_publisher_.publish(goal_status);
		ROS_INFO_COND(do_debug_strategy_, "%s", ss.str().c_str());
	}

	StrategyFn() {
		assert(ros::param::get("~do_debug_strategy", do_debug_strategy_));
		current_strategy_pub_ = nh_.advertise<std_msgs::String>("current_strategy", 1, true /* latched */);
		strategy_status_publisher_ = nh_.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);
	}

public:
	static const string& currentGoalName() {
		if (g_goal_stack_.empty()) return g_empty_string;
		else return g_goal_stack_.back().goal_name;
	}

	static const string& currentGoalParam() {
		if (g_goal_stack_.empty()) return g_empty_string;
		else return g_goal_stack_.back().goal_param;
	}

	static bool goalStackEmpty() { return g_goal_stack_.empty(); }

	static RESULT_T lastGoalResult() { return g_last_goal_result_; }

	static void popGoal() {
		g_goal_stack_.pop_back();
	}

	static void pushGoal(string name, string param = "") {
		g_goal_stack_.push_back(GOAL_T(name, param));
	}

	static const GPS_POINT& currentGpsPoint() { return g_point_stack.back(); }
	static void popGpsPoint() { g_point_stack.pop_back(); }
	static void pushGpsPoint(GPS_POINT gps_point) { g_point_stack.push_back(gps_point); }

	// Name of strategy.
	virtual string name() = 0;

	static string resultToString(RESULT_T result) {
		if ((result <= UNUSED_START) || (result >= UNUSED_END)) {
			return "BAD result code";
		} else {
			return string(RESULT_STR[result]);
		}
	}

	static RESULT_T setGoalResult(RESULT_T result) {
		g_last_goal_result_ = result;
		return result;
	}

	// Perform strategy.
	virtual RESULT_T tick() = 0;

	// Get string needed to initiate the goal.
	virtual string goalName() = 0;
};

#endif
