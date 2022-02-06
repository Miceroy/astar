///=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
/// MIT License
///
/// Copyright (c) 2020 Mikko Romppainen
/// 
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <algorithm>
#include <map>
#include <vector>
#include <stdint.h>
#include <math.h>
#include <functional>
#include "astar.h"
#include <string>



// Typedef for Position and Action
class State : public std::vector<int>{
public:
    State() : std::vector<int>() {

    }

    State(const State& o) : std::vector<int>(o) {

    }

    State(const std::initializer_list<int>& in) : std::vector<int>(in) {

    }
};

typedef State(*Action)(State);

template<typename IsEndStateType, typename GetGCostType, typename GetHCostType>
std::vector<Action> plan(const State& start, const IsEndStateType& isEndState, const std::vector<Action>& actions, GetGCostType getGCost, GetHCostType getHCost) {
    // A* lambdas for goap planner

    // Lambda for checking is action legal in current state.
    auto guard = [](const State& state, const Action &action) -> bool {
        // Simple guard: Make action and if new states has negative values (running out of resource, return false.
        const State newState = action(state);
        for(auto s : newState) {
            if(s < 0) {
                return false;
            }
        }

        return true;
    };

    astar::State<State, Action> state(start);
    while (false == astar::aStar(state, isEndState, actions, getHCost, getGCost, guard)) {
    }
    return state.path;
}

enum Items {
    TIME,
    ENERGY,
    MONEY,
    FOOD,
};


State buyFood(State state) {
    state[TIME] -= 30;
    state[ENERGY] -= 100;
    state[MONEY] -= 100;
    state[FOOD] += 7;
    return state;
}

State work(State state) {
    state[TIME] -= 60;
    state[ENERGY] -= 400;
    state[MONEY] += 100;
    return state;
}

State eat(State state) {
    state[TIME] -= 15;
    state[FOOD] -= 1;
    state[ENERGY] += 1200;
    return state;
}

// Actions to move to: left, right up or down.
const std::vector<Action> actions = {
	//buyFood, work, eat
	work, eat, buyFood
};


const std::vector<std::string> actionIndexToActionName = {
	{"Work"},
	{"Eat"},
	{"Buy food"}
};

template<typename GetGCostType>
void printPlan(State now, const std::vector<Action>& path, GetGCostType getGCost, const std::vector<Action>& actions) {


    if (path.size() > 0) {
        printf("\nPath found (%d actions):\n", (int)path.size() );
        for (auto action : path) {
             printf("(");
            for(size_t i=0; i<now.size(); ++i) {
                std::string fmt = "%d";
                if( i!=0 ) {
                    fmt = std::string(", ") + fmt;
                }
                printf(fmt.c_str(), now[i]);
            }

			std::string actionName;
			for (size_t i = 0; i < actions.size(); ++i) {
				if (actions[i] == action) {
					actionName = actionIndexToActionName[i];
					break;
				}
			}
            printf(") - %s - > (", actionName.c_str());
            now = action(now);

            for(size_t i=0; i<now.size(); ++i) {
                std::string fmt = "%d";
                if( i!=0 ) {
                    fmt = std::string(", ") + fmt;
                }
                printf(fmt.c_str(), now[i]);
            }
            printf(")\n");
        }
    }
    else {
        printf("\nPath not found!\n");
    }
}

int main() {

    // Start and end states: Try to earn money.
    State startState; startState.resize(4);
    startState[TIME] = 2000;
    startState[ENERGY] = 0;
    startState[MONEY] = 0;
    startState[FOOD] = 5;

	State goalState; goalState.resize(4);
	goalState[MONEY] = 2000;

    auto isEndState = [&goalState](const State& state) -> bool {
        return state[MONEY] >= goalState[MONEY];
    };



    // Lambda for getting a cost of an action.
    auto getGCost = [](const State& state, const Action& action)-> float {
        const State newState = action(state);
		// Total cost is how much time and energy was spend?
		float timeCost = (float)state[TIME] - newState[TIME];
		float energyCost = (float)state[ENERGY] - newState[ENERGY];
		if (energyCost < 0) energyCost = 0;
        return timeCost + energyCost;
    };

    // Lambda for getting a heurestic cost between given state and gloal state.
    auto getHCost = [](const State& state) -> float {
        return 0;
    };

    auto result = plan(startState, isEndState, actions, getGCost, getHCost);

    printPlan(startState, result, getGCost, actions);

	printf("\nPress ENTER to continue...\n");
	getchar();
	return 0;
}

