#include "victoria_navigation/strategy_context.h"

StrategyContext& StrategyContext::singleton() {
	static StrategyContext singleton_;
	return singleton_;
}
