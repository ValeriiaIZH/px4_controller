# Readme 

## The Latest Start
- Open directory
```
cd UAV_Swarm_gazebo/catkin_ws/src/px4_controller/rrt_pruning_smoothing/Code
```
At first, change the path of working directories. Next, check for the presence of the mission JSON file in coverage_planner/scripts and px4_controller/rrt_pruning_smoothing/Code
- Run bash script (for scenario №1 with coverage path planning). Then, check the path to the mission JSON file in coverage_planner/scripts/initpose.py and px4_controller/rrt_pruning_smoothing/python rrt_(start/reset/land)_test.py
```
./path_planning_rrt_cpp.sh
```
As a result 2 files: global_path_interface.txt without smoothing and global_path_interface_smooth.txt with smoothing. These trajectories are already transmitted to the flight controller and to the interface for display on the map.

- Or run python script (for scenario №2 without coverage path planning)
At first, change the path of working directories. Next, check for the presence of the mission JSON files(Map2.json, Obs.json, Targets.json) in px4_controller/rrt_pruning_smoothing/Code
```
python path_planning_rrt.py
```
As a result 2 files: path_global_scenario2.txt without smoothing and path_global_scenario2_smooth.txt with. These trajectories are already transmitted to the flight controller and to the interface for display on the map.


## Start (old version 1)
```
roslaunch px4 posix_sitl.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/asd.world
```

- Run the script `../src/px4_controller/rrt-uav/Code/main.py` to generate the path.

- To run the simulation run the following command in a new ros terminal.
```
source ~/src/catkin_ws/devel/setup.bash
rosrun px4_controller position_control.py 
```
## Start (old version 2 with topic's)
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
```
cd UAV_Swarm_gazebo/Autopilot/
DONT_RUN=1 make px4_sitl_default gazebo
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```

- Run parameter listener
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash  
rosrun px4_controller parameter_listener.py
```
- Run start and goal point talker
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash  
rosrun px4_controller start_goal_point_talker.py
```
- Run obstacles talker
You need input:
 	- Quantity obstacles
 	- Plot limit's
 	- Obstacle coordinates and radius
```
source ~/UAV_Swarm_gazebo/catkin_ws/devel/setup.bash
rosrun px4_controller obstacles_talker.py 
```


