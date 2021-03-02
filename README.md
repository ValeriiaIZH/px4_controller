# Readme 

```
roslaunch px4 posix_sitl.launch world:=$(pwd)/Tools/sitl_gazebo/worlds/asd.world
```

- Run the script `../src/px4_controller/rrt-uav/Code/main.py` to generate the path.

- To run the simulation run the following command in a new ros terminal.
```
source ~/src/catkin_ws/devel/setup.bash
rosrun px4_controller position_control.py 
```
