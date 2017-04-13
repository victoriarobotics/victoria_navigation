#ifndef __VICTORIA_NAVIGATION_STRATEGY_FN
#define __VICTORIA_NAVIGATION_STRATEGY_FN

#include <ros/ros.h>
#include <actionlib_msgs/GoalStatus.h>
#include <map>
#include <std_msgs/String.h>
#include <string>
#include <vector>

class StrategyFn {
public:
	typedef struct GOAL_T {
		std::string 	goal_name;	// Name of goal.
		std::string 	goal_param;	// Optional parameter for goal.
		GOAL_T(std::string name, std::string param) : goal_name(name), goal_param(param) {}
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

	enum RESULT_T {
		UNUSED_START = 0,	// Do not use, must be first element.
		FAILED,				// Strategy failed, do not continue.
		FATAL,				// Something is fatally wrong.
		INACTIVE,			// Strategy is not active.
		RUNNING,			// Strategy is in progress.
		SUCCESS,			// Strategy succeeded, continue on.
		UNUSED_END			// Do not use, must be last element.
	};

protected:
	static const std::string G_RESULT_STR_[];		// Map RESULT_T enum to string.
	static std::vector<GOAL_T> g_goal_stack_;		// Stack of goals to be solved.
	static std::vector<GPS_POINT> g_point_stack_;	// Stack of goal GPS points, related to g_goal_stack.
	static const std::string G_EMPTY_STRING_;		// Singleton empty string.
	static const std::string G_NO_GOAL_NAME_;		// Singleton string for name of non-existing goal.
	static RESULT_T g_last_goal_result_;			// Last result of current goal tick().

	// ROS node handle.
	ros::NodeHandle nh_;

	// Parameters.
	bool do_debug_strategy_;			// Emit info traces to help debug code.

	// Publishers.
	ros::Publisher current_strategy_pub_;
	ros::Publisher strategy_status_publisher_;

	void publishStrategyProgress(const std::string& strategy_name, const std::string& progress) {
		static std::map<std::string, std::string> last_progress_message;
		actionlib_msgs::GoalStatus 	goal_status;
		std::ostringstream ss;

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
	static const std::string& currentGoalName() {
		if (g_goal_stack_.empty()) return G_NO_GOAL_NAME_;
		else return g_goal_stack_.back().goal_name;
	}

	static const std::string& currentGoalParam() {
		if (g_goal_stack_.empty()) return G_EMPTY_STRING_;
		else return g_goal_stack_.back().goal_param;
	}

	// Are all goals complete?
	static bool goalStackEmpty() { return g_goal_stack_.empty(); }

	// Last reported result for the current goal.
	static RESULT_T lastGoalResult() { return g_last_goal_result_; }

	// Complete the current goal and return to the previous goal.
	static void popGoal() {
		g_goal_stack_.pop_back();
	}

	// Replace the current goal with a new one.
	static void pushGoal(const std::string& name, const std::string& param = G_EMPTY_STRING_) {
		g_last_goal_result_ = INACTIVE;	// If a new goal starts, ignore the previous goal result.
		g_goal_stack_.push_back(GOAL_T(name, param));
	}

	static const GPS_POINT& currentGpsPoint() { return g_point_stack_.back(); }
	static void popGpsPoint() { g_point_stack_.pop_back(); }
	static void pushGpsPoint(const GPS_POINT& gps_point) { g_point_stack_.push_back(gps_point); }

	// Name of strategy.
	virtual const std::string& name() = 0;

	static const std::string& resultToString(RESULT_T result) {
		static const std::string bad_result_code = "BAD result code";
		if ((result <= UNUSED_START) || (result >= UNUSED_END)) {
			return bad_result_code;
		} else {
			return G_RESULT_STR_[result];
		}
	}

	static RESULT_T setGoalResult(RESULT_T result) {
		g_last_goal_result_ = result;
		return result;
	}

	// Singleton constructor
	static StrategyFn& singleton();

	// Perform strategy.
	virtual RESULT_T tick() = 0;

	// Get string needed to initiate the goal.
	virtual const std::string& goalName() = 0;
};

#endif  // __VICTORIA_NAVIGATION_STRATEGY_FN
