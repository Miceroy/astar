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



// Typedef for Position and Action
typedef std::pair<int, int> Position;
typedef std::function<Position(const Position&)> Action;


template<typename GridType>
std::vector<Action> findPath(const GridType& level, const Position& start, const Position& end) {
    // Typedef for Action
    typedef std::function<Position(const Position&)> Action;

    // A* for tile based level

    // Actions to move to: left, right up or down.
    std::vector<Action> actions = {
            [](const Position& pos) -> Position { return Position(pos.first - 1, pos.second); },
            [](const Position& pos) -> Position { return Position(pos.first + 1, pos.second); },
            [](const Position& pos) -> Position { return Position(pos.first, pos.second - 1); },
            [](const Position& pos) -> Position { return Position(pos.first, pos.second + 1); }
    };

    // Lambda for getting a heurestic cost between two states.
    auto getHCost = [&end](const Position& pos) -> float {
        // Euclidian distance
        float dx = float(end.first - pos.first);
        float dy = float(end.second - pos.second);
        return sqrtf(dx*dx + dy*dy);
    };

    // Lambda for getting a cost of an action.
    auto getGCost = [](const Position& pos, const Action& action)-> float {
        // Cost is always one.
        return 1.0f;
    };

    // Lambda for checking is given position legal
    auto isLegalPosition = [&level](const Position& pos) -> bool {
        if (pos.first < 0 || pos.second < 0 || pos.second >= level.size() || pos.first >= level[pos.second].size() ) return false;
        return !level[pos.second][pos.first];
    };

    // Lambda for checking is action legal in current state.
    auto isLegalAction = [isLegalPosition](const Position& pos, const Action &action) -> bool {
        return isLegalPosition(action(pos)); // For simplicity, call the action and check if the end position is legal.
    };

    auto isEndPosition = [&end](const Position& pos) -> bool {
        return pos == end;
    };

    astar::State<Position,Action> state(start);
    while (false == astar::aStar(state, isEndPosition, actions, getHCost, getGCost, isLegalAction)) {
    }
    return state.path;
}

int main() {
    typedef std::vector< std::vector<int> > GridType;

    // Game level
    GridType level = {
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1},
		{1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1},
		{1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1},
		{1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
		{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
	};/*
    std::vector<int> level = {
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
	};*/



	// Start and end positions
	Position start(1, 1);
    Position end(11, 7);

	// Lambda for printing level
	auto printLevel = [&level]() {
        for (auto y : level) {
            for (auto v : y) {
                printf("%d", v);
            }
			printf("\n");
		}
	};

    // First print the level

	printf("Search level:\n");
	printLevel();

    auto result = findPath(level, start, end);

    if (result.size() > 0) {
		printf("\nPath found:\n");
        for (auto action : result) {
            level[start.second][start.first] = 2; // Mark path to the level
            start = action(start);
		}
		printLevel();
	}
	else {
		printf("\nPath not found!\n");
	}

	printf("\nPress ENTER to continue...\n");
	getchar();
	return 0;
}

