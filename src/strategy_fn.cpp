#include "victoria_navigation/strategy_fn.h"

const std::string StrategyFn::G_RESULT_STR_[] = {
		"UNUSED_START",
		"FAILED",
		"FATAL",
		"INACTIVE",
		"RUNNING",
		"SUCCESS",
		"UNUSED_END"
};

const std::string StrategyFn::G_EMPTY_STRING_ = "";
std::vector<StrategyFn::GOAL_T> StrategyFn::g_goal_stack_;
std::vector<StrategyFn::GPS_POINT> StrategyFn::g_point_stack_;
StrategyFn::RESULT_T StrategyFn::g_last_goal_result_;
