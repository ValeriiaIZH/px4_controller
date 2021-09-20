#!/bin/bash

# At first, change the path of working directories

# Next, check for the presence of the mission JSON file 
# in coverage_planner/scripts and px4_controller/rrt_pruning_smoothing/Code

# Then, check the path to the mission JSON file in coverage_planner/scripts/initpose.py 
# and px4_controller/rrt_pruning_smoothing/python rrt_(start/reset/land)_test.py

# Start ROS
gnome-terminal --working-directory =/home/valeriia/UAV_Swarm_gazebo -e 'roscore'&
sleep 0.5

# Start ROS publisher, which publishes data from mission JSON file
gnome-terminal --working-directory=/home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/coverage_planner/scripts/ -- bash -c 'source /home/valeriia/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash; python initpose.py ; exec bash'
#sleep 5 

# Start ROS node, which reads published data
gnome-terminal -- bash -c 'source /home/valeriia/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash; rosrun coverage_planner coverage_planner_node; exec bash'
sleep 15

# Sequential trajectory calculation
cd /home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/px4_controller/rrt_pruning_smoothing/Code
sleep 5
python rrt_start_test.py
python rrt_reset_test.py
python rrt_land_test.py

# Combining all trajectories into one and creating two files with a common third: 
# 1) without smoothing, 2) with smoothing. These trajectories are already transmitted 
# to the flight controller and to the interface for display on the map.
cd /home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/coverage_planner/scripts
python global_path_merge_test.py      # As a result file global_path_interface.txt without smoothing
python global_path_merge_smooth.py    # As a result file global_path_interface_smooth.txt with smoothing
