#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

# At first, change the path of working directories

# Next, check for the presence of the mission JSON file 
# in px4_controller/rrt_pruning_smoothing/Code

# Then, check the path to the mission JSON file in 
# px4_controller/rrt_pruning_smoothing/python rrt_(start/reset/land)_scenario2.py

# Sequential trajectory calculation
os.system("python rrt_start_scenario2.py")
os.system("python rrt_reset_scenario2.py")
os.system("python rrt_land_scenario2.py")

os.system("python global_path_merge_scenario2.py")          # As a result file path_global_scenario2.txt without smoothing
os.system("python global_path_merge_scenario2_smooth.py")   # As a result file path_global_scenario2_smooth.txt with smoothing
