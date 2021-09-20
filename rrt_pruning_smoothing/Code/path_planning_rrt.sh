#!/bin/bash

# At first, change the path of working directories

# Next, check for the presence of the mission JSON file 
# in px4_controller/rrt_pruning_smoothing/Code

# Then, check the path to the mission JSON file in 
# px4_controller/rrt_pruning_smoothing/python rrt_(start/reset/land)_scenario2.py

# Sequential trajectory calculation
cd /home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/px4_controller/rrt_pruning_smoothing/Code
sleep 5
python rrt_start_scenario2.py
python rrt_reset_scenario2.py
python rrt_land_scenario2.py

# Combining all trajectories into one and creating two files with a common third: 
# 1) without smoothing, 2) with smoothing. These trajectories are already transmitted 
# to the flight controller and to the interface for display on the map.
cd /home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/coverage_planner/scripts
python global_path_merge_scenario2.py           # As a result file path_global_scenario2.txt without smoothing
python global_path_merge_scenario2_smooth.py    # As a result file path_global_scenario2_smooth.txt with smoothing
