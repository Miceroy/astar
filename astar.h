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
#pragma once
#include <stdint.h>
#include <map>
#include <vector>
#include <algorithm>
#include <math.h>

namespace astar {
    // Struct for plan node
    template<typename StateType, typename ActionType>
    class PlanNode {
    public:
        PlanNode(const StateType& _state, const ActionType& _action, float g, float h, PlanNode* prev) : state(_state), action(_action), prevNode(prev), H(h), G(g) {
        }

        const StateType state;
        const ActionType action;
        const PlanNode* prevNode;
		const float H;
		const float G;

        float getTotalG() const {
            // Just compute previous nodes and return cost to travel from start to this node
            float res = 0.0f;
            const PlanNode* scan = this;
            while (scan->prevNode) {
                res += scan->G;
                scan = scan->prevNode;
            }
            return res;
        }

        float getF() const {
            // F = total G + H
            return getTotalG() + H;
        }
    };

    template<typename StateType, typename ActionType>
    struct State {
        State(const StateType& startState) : start(startState), openList(), closedList(), path() {
            openList.push_back(new astar::PlanNode<StateType, ActionType>(start, 0, 0, 0, 0)); // Add start node to open list
        }
        
        ~State() {
            for (auto o : openList) {
                delete o;
            }

            for (auto o : closedList) {
                delete o.second;
            }
        }
        // Open list
        StateType start;
        std::vector<PlanNode<StateType,ActionType>*> openList;
        std::map<StateType, PlanNode<StateType, ActionType>*> closedList;
        std::vector<ActionType> path;
    private:
        State();
        State(const State&);
        State& operator=(const State&);
    };



    template<typename StateType, typename ActionType, typename IsEndStateType, typename GetHCostType, typename GetGCostType, typename GuardType>
    bool aStar(State<StateType,ActionType>& state, const IsEndStateType& isEndState, const std::vector<ActionType>& actions, GetHCostType getHCost, GetGCostType getGCost, GuardType isLegalAction) {
        
        // Lambda for getting adjacent nodes. Gets current position as a parameter.
        // Returns new positions, where we can move from current position.
        auto getNextStates = [isLegalAction, actions](const StateType& pos) -> std::vector< std::pair<StateType,ActionType> > {
            // Compute new states from old state by calling the action function. If new state is legal,
            // Add it to the results.
            typedef std::pair<StateType,ActionType> PairType;
            std::vector<PairType> res;
            for(auto action : actions) {
                if(isLegalAction(pos, action)) {
                    res.push_back( PairType(action(pos), action) );
                }
            }
            return res;
        };

        // A*
        const PlanNode<StateType,ActionType>* result = 0; // Save possible A* result here
        if (state.openList.size() > 0) {
            // Find node with smallest F cost to N.
            PlanNode<StateType, ActionType>* N = state.openList[0];
            int nF = N->getF();
            size_t index = 0;
            for (size_t i = 0; i < state.openList.size(); ++i) {
                auto newnF = state.openList[i]->getF();
                if (newnF < nF) {
                    N = state.openList[i];
                    nF = newnF;
                    index = i;
                }
            }

            state.openList.erase(state.openList.begin() + index, state.openList.begin() + index + 1); // Remove it from open list
            state.closedList[N->state] = N; // Add it to the closed list

            // Has reached destination?
            if (isEndState(N->state)) {
                result = N;
            }
            else {
                // Get neighbour nodes, where we can walk
                for (auto nextAction : getNextStates(N->state)) {
                    auto newState = nextAction.first;
                    auto action = nextAction.second;
                    // If already at the closed list (seen states), ignore it
                    if (state.closedList.find(newState) != state.closedList.end()) {
                        continue;
                    }

                    // If already at open list?
                    auto it = std::find_if(state.openList.begin(), state.openList.end(), [newState](const PlanNode<StateType, ActionType>* n) -> bool {
                        return n->state == newState;
                    });

					const auto G = getGCost(N->state, nextAction.second);
					const auto H = getHCost(newState);
					auto* newN = new PlanNode<StateType, ActionType>(newState, action, G, H, N); // We came to N_-node from N-node.

                    if (it != state.openList.end()) {
                        // Found from open list, if new F is smaller than prev route to this position, reset prev node from this route
                        if ((*it)->getTotalG() > newN->getTotalG()) {
							(*it) = newN;
						}
						else {
							// Delete N_, because we do not need it anymore
							delete newN;
						}
                    }
                    else {
                        state.openList.push_back(newN); // Add new node to the open list
                    }
                }
            }
        } // End - if (state.openList.size() > 0)

        // If no result and no nodes left, stop by returning true.
        if (result == 0) {
            return state.openList.size() == 0;
        }

#if _DEBUG
		printf("aStar found path of total cost: %.2f\n", result->getTotalG());
#endif
        while (result != 0) {
            if(result->action) {
                state.path.insert(state.path.begin(), result->action);
            }
            result = result->prevNode;
        }
        return true; // Stop
    }

} // End - namespace astar

