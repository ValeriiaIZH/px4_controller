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
import json
import numpy as np 
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import rrt
import node_rrt
# import univ
import utils 
import obstacles as obs
import time

import pickle
import rospy
from geometry_msgs.msg import Point
import parameter_listener as prm
#import path_talker as pt
 
X_LIM = (prm._lim_x, prm.lim_x)
Y_LIM = (prm._lim_y, prm.lim_y)

global my_rrt_path_coords
my_rrt_path_coords = []

def rrtPlannedPath(start_node, goal_node, robot_radius, plotter, write=False):

	step_size = robot_radius * 2

	rrt_nodes = {start_node.getXYCoords(): start_node}
	
	step_node = start_node

	itr = 0
	while (not utils.sameRegion(step_node, goal_node, prm.c_goal_reach_thresh)) and (itr < prm.c_max_iter):
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
	if utils.sameRegion(step_node, goal_node, prm.c_goal_reach_thresh):
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
	start_time  = time.time()

	prm.main()

	print(X_LIM)
	#possition_listener()
	if (prm.start_pose_x == None) or (prm.start_pose_y == None):
		rospy.loginfo("Problem with start coordinates!")
		rospy.loginfo("Start x: '%s' and y: '%s' coordinates", str(prm.start_pose_x), str(prm.start_pose_y))
		sys.exit(0)
	elif  (prm.goal_pose_x == None) or (prm.goal_pose_y == None):
		rospy.loginfo("Problem with goal coordinates!")
		rospy.loginfo("Goal x: '%s' and y: '%s' coordinates", str(prm.goal_pose_x), str(prm.goal_pose_y))
		sys.exit(0)
	start_node = node_rrt.Node_rrt(current_coords=(prm.start_pose_x, prm.start_pose_y), parent_coords=None, distance=0)
	goal_node = node_rrt.Node_rrt(current_coords=(prm.goal_pose_x, prm.goal_pose_y), parent_coords=None, distance=0)

	fig, ax = plt.subplots()
	ax.set_xlim(prm._lim_x, prm.lim_x)
	ax.set_ylim(prm._lim_y, prm.lim_y)
	fig.gca().set_aspect('equal', adjustable='box')
	utils.plotPoint(start_node.getXYCoords(), ax, radius=0.06, color='cyan') 	# start
	utils.plotPoint(goal_node.getXYCoords(), ax, radius=0.06, color='magenta') 	# end

	#obs.testMain()
	obs.generateMap(ax)

	plt.ion()
	rrt_path, _, itr = rrtPlannedPath(start_node, goal_node, robot_radius = prm.c_drone_radius, plotter=ax, write=False)
	if rrt_path is not None:
		utils.plotPath(rrt_path, plotter=ax)

	rrt_path_coords = utils.convertNodeList2CoordList(node_list=rrt_path)
	path_length = utils.path_length_meters(rrt_path_coords)
	try:
		capacity = float(str(prm.capacity)[5:])
		max_length = float(str(prm.max_length)[5:])
		max_uav_path = capacity*max_length
		if max_uav_path < path_length:
			print("\nWarning!")
			print("Path cannot be overcomed!")
		else:
			print("Path can be overcomed!")
	except:
		print("Capacity and maximum path lenght was not entered!")

	finish_time = time.time()
	result = finish_time - start_time
	print("Program time: " + str(result) + " seconds.")

	# np.save(file='rrt_path_nodes.npy', arr=rrt_path)
	# np.save(file='rrt_path_coords.npy', arr=rrt_path_coords)

	rrt_path_coords = utils.convertNodeList2CoordList(node_list=rrt_path)
	

	with open('path.txt', 'w') as f:
		f.write(json.dumps(rrt_path_coords))	
	plt.ioff()
	plt.show()
	sys.exit

	
if __name__ == '__main__':	
	main()

