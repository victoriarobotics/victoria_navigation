#include "victoria_navigation/strategy_fn.h"

const char* StrategyFn::RESULT_STR[] = {
		"UNUSED_START",
		"FAILED",
		"FATAL",
		"RESTART_LOOP",
		"RUNNING",
		"SUCCESS",
		"UNUSED_END"
};

const string StrategyFn::g_empty_string = "";
vector<StrategyFn::GOAL_T> StrategyFn::g_goal_stack_;
vector<StrategyFn::GPS_POINT> StrategyFn::g_point_stack;
StrategyFn::RESULT_T StrategyFn::g_last_goal_result_;
