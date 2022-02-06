cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# Project
project(AStar)

# Exampels
add_executable(example_path_finding astar.h main_path_finding_from_grid.cpp)
add_executable(example_goap astar.h main_goal_oriented_action_planning.cpp)

