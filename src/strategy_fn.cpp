#include "victoria_navigation/strategy_fn.h"

const char* StrategyFn::RESULT_STR[] = {
		"UNUSED_START",
		"FAILED",
		"FATAL",
		"INACTIVE",
		"RUNNING",
		"SUCCESS",
		"UNUSED_END"
};

const std::string StrategyFn::g_empty_string = "";
std::vector<StrategyFn::GOAL_T> StrategyFn::g_goal_stack_;
std::vector<StrategyFn::GPS_POINT> StrategyFn::g_point_stack_;
StrategyFn::RESULT_T StrategyFn::g_last_goal_result_;
