#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
from __future__ import print_function, division 
import sys
import math
import heapq
import random 
import numpy as np 
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import json

import rrt
import node_rrt
# import univ
import path_pruning
import utils 
import obstacles as obs
import time


X_LIM = (-15000,15000)
Y_LIM = (-15000,15000)

MAX_ITER = 50000
STEP_SIZE = 400
GOAL_REACH_THRESH = 800	

DRONE_RADIUS = 100


def rrtPlannedPath(start_node, goal_node, robot_radius, plotter, write=False):
	step_size = robot_radius * 2

	rrt_nodes = {start_node.getXYCoords(): start_node}
	
	step_node = start_node

	itr = 0
	while (not utils.sameRegion(step_node, goal_node, GOAL_REACH_THRESH)) and (itr < MAX_ITER):
		# print("Iteration number:", itr)
		itr += 1

		# get random node in the direction of the goal node 40% of the times.
		rand_node = rrt.getRandomNode(x_lim=X_LIM, y_lim=Y_LIM, curr_node=start_node, goal_node=goal_node, goal_probability=0.6)
		if plotter is not None:
			utils.plotPoint(rand_node.getXYCoords(), plotter, radius=0.03, color='red')

		closest_node, _ = rrt.findClosestNode(rrt_nodes.values(), rand_node)

		step_node = rrt.getStepNode(closest_node, rand_node, step_size)
		if plotter is not None:
			utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='blue')
			cn_x, cn_y = closest_node.getXYCoords()
			sn_x, sn_y = step_node.getXYCoords()
			plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='blue')
			# plt.show()
			# plt.pause(0.5)

		if rrt.hitsObstacle(start_node=closest_node, goal_node=step_node, step_size=(robot_radius/2)):
			if plotter is not None:
				utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='red')
				cn_x, cn_y = closest_node.getXYCoords()
				sn_x, sn_y = step_node.getXYCoords()
				plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='red')
				# plt.show()
				# plt.pause(0.5)
			continue

		if plotter is not None:
			utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='green')
			cn_x, cn_y = closest_node.getXYCoords()
			sn_x, sn_y = step_node.getXYCoords()
			plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='green')
			# plt.show()
			# plt.pause(0.5)

		rrt_nodes.update({step_node.getXYCoords(): step_node})

		if plotter is not None:
			plt.show()
			plt.pause(0.05)

		if write:
			plt.savefig('./frames/' + str(itr) + '.png')

	# Reached Goal
	if utils.sameRegion(step_node, goal_node, GOAL_REACH_THRESH):
		print("Reached Goal!")
		print("Number of iterations:", itr)

		goal_node.parent_coords = closest_node.getXYCoords()
		goal_node.distance = utils.euclideanDistance(point_1=closest_node.getXYCoords(), point_2=goal_node.getXYCoords())
		rrt_nodes.update({goal_node.getXYCoords(): goal_node})
		# rrt_nodes.append(goal_node)

		path = rrt.backtrack(rrt_nodes, goal_node)

		print("path:", len(path), "rrt_nodes:", len(rrt_nodes))

		return (path, rrt_nodes, itr)

	return (None, None, None)


def main():

	json_data = []
	with open('Map2.json', 'r') as f: 
		json_data = json.load(f) 
		start_point_ = json_data.get('start_point')
		#centroid_ = json_data.get('centroid')
	'''json_data_2 = []
	with open('Scenario2.json', 'r') as d: 
		json_data_2 = json.load(d) 
		target_point = json_data_2.get('centroid')'''	

	x = [-957.3915945077315, -910.8959478922188, -948.4347058273852, -875.1556231221184, -845.2041011163965, -858.6145041380078, -919.3042095946148, -942.3035844964907, -933.9325257679448, -889.955683006905]
	y = [-855.0138749238104, -895.5183857586235, -921.3926183544099, -947.264425364323, -858.3795844987035, -861.4421726837754, -895.0775583777577, -844.3298955513164, -922.8892891705036, -887.7070486117154]
	centroid = [sum(x) / len(x), sum(y) / len(y)]
	print(centroid)
	start_point = start_point_.values()
	#target_point = centroid_.values()
	print(type(start_point))
	print(centroid)
	start_time  = time.time()
	start_node = node_rrt.Node_rrt(current_coords=(start_point[0],start_point[1]), parent_coords=None, distance=0)
	goal_node = node_rrt.Node_rrt(current_coords=(centroid[0],centroid[1]), parent_coords=None, distance=0)

	fig, ax = plt.subplots()
	ax.set_xlim(-15000, 15000)
	ax.set_ylim(-15000, 15000)
	fig.gca().set_aspect('equal', adjustable='box')
	utils.plotPoint(start_node.getXYCoords(), ax, radius=0.06, color='cyan') 	# start
	utils.plotPoint(goal_node.getXYCoords(), ax, radius=0.06, color='magenta') 	# end

	obs.generateMap(ax)

	plt.ion()
	rrt_path, _, itr = rrtPlannedPath(start_node, goal_node, STEP_SIZE, plotter=ax, write=False)
	if rrt_path is not None:
		utils.plotPath(rrt_path, plotter=ax)

	finish_time = time.time()
	result = finish_time - start_time
	print("Program time: " + str(result) + " seconds.")

	plt.ioff()

	path_co = np.array(utils.convertNodeList2CoordList(rrt_path))

	rrt_prune_smooth_path_coords=path_pruning.prunedPath(path=path_co, radius=DRONE_RADIUS, clearance=(DRONE_RADIUS))
	rrt_prune_smooth_path_coords= np.array(rrt_prune_smooth_path_coords[::-1])
	plt.plot(rrt_prune_smooth_path_coords[:,0],rrt_prune_smooth_path_coords[:,1],'cyan')
	
	plt.show()

	rrt_path_coords = utils.convertNodeList2CoordList(node_list=rrt_path)
	with open('/home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/px4_controller/rrt_pruning_smoothing/Code/path_rrt_test_scenario2.txt', 'w') as fp:
		fp.write('\n'.join('%s %s' % x for x in rrt_path_coords))

	with open('/home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/px4_controller/rrt_pruning_smoothing/Code/path_rrt_test__scenario2_smooth.txt', 'w') as dp:
		dp.write('\n'.join('%s %s' % x for x in list(map(tuple, rrt_prune_smooth_path_coords))))

if __name__ == '__main__':
	main()
