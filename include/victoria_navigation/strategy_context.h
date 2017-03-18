#ifndef __STRATEGY_CONTEXT
#define __STRATEGY_CONTEXT

#include <map>
#include <string>

using namespace std;

class StrategyContext {
private:
	// Singleton pattern.
	StrategyContext() {}
	StrategyContext(StrategyContext const&) {};
	StrategyContext& operator=(StrategyContext const&) {}

public:
	static StrategyContext& singleton();

	map<string, string> blackboard;

};

#endif
