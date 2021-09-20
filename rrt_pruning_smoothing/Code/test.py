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

'''import json
import matplotlib.pyplot as plt
json_data = []
#start_point = []
x_arr_obs = []
y_arr_obs = []
with open('Obs.json', 'r') as f: 
    json_data = json.load(f) 
    points_arr = json_data.get('points')
for i in range(len(points_arr)):
        points_arr_ = points_arr[i]
        x = points_arr_[0]
        y = points_arr_[1]
        x_arr_obs.append(x)
        y_arr_obs.append(y)

obstacle = zip(x_arr_obs,y_arr_obs)
print(obstacle)'''

'''x = [12595.426636713557, 12628.819110970944, 12620.887485932559, 12563.654373116791, 12649.810772065073, 12590.579886271618, 12551.79453928303, 12605.608974876814, 12595.183648529463, 12615.256333830766]
y = [-10528.186294430867, -10539.132578887977, -10577.25304689072, -10556.820479613729, -10597.169939996675, -10527.812698546797, -10536.931147849187, -10606.115751585923, -10563.024337255396, -10515.654916515574]
centroid = (sum(x) / len(x), sum(y) / len(y))
print(centroid)


plt.plot([1, 50],[100, 5])
plt.ylabel('Battery charge, %')
plt.xlabel('Время, min')
plt.title('Multi-rotor battery discharge graph')
plt.grid(True)
plt.show()

plt.plot([1, 600],[100, 5])
plt.ylabel('Battery charge, %')
plt.xlabel('Time, min')
plt.title('Fixed-wing fuel consumption graph')
plt.grid(True)
plt.show()'''


#(12601.702176159062, -10554.810119157284)

