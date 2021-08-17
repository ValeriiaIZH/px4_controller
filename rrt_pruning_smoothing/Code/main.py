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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
from __future__ import print_function, division
import sys
import math
import heapq
import random
from tokenize import Bracket 
import numpy as np 
import matplotlib.pyplot as plt
sys.dont_write_bytecode = True

import rrt
import node_rrt
import utils 
import path_pruning
import obstacles as obs

import rospkg 
import rospy
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from pickle import TRUE
import rospy
from geometry_msgs.msg import Point
import sys
import parameter_listener as prm
#import timeit


#code_to_test = """



#"""
#elapsed_time = timeit.timeit(code_to_test, number=100)/100
#print(elapsed_time)
"""
my norm param
MAX_ITER = 5000
STEP_SIZE = 0.06
GOAL_REACH_THRESH = 4	

DRONE_RADIUS = 2
"""

X_LIM = (prm._lim_x, prm.lim_x)
Y_LIM = (prm._lim_y, prm.lim_y)

MAX_ITER = 5000
STEP_SIZE = 0.1
GOAL_REACH_THRESH = 2

DRONE_RADIUS = 0.3

"""start_pose_x = None
start_pose_y = None
goal_pose_x = None
goal_pose_y = None

def start_pos_callback(data): 
    global start_pose_x
    global start_pose_y 
    start_pose_x = data.x
    start_pose_y = data.y
	#loginfo for test
    #rospy.loginfo("I heard start %s ", str(start_pose_x) + " and " + str(start_pose_y))
def goal_pos_callback(data): 
    global goal_pose_x
    global goal_pose_y  
    goal_pose_x = data.x
    goal_pose_y = data.y
	#loginfo for test
    #rospy.loginfo("I heard goal %s ", str(goal_pose_x) + " and " + str(goal_pose_y))
def possition_listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('possition_listener', anonymous=True)
        rospy.Subscriber('/start_point/position', Point, start_pos_callback)
        rospy.Subscriber('/goal_point/position', Point, goal_pos_callback)
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        rospy.sleep(5)"""

def rrtPlannedPath(start_node, goal_node, robot_radius, plotter, write=False):
	step_size = robot_radius * 2

	# rrt_nodes = [start_node]
	rrt_nodes = {start_node.getXYCoords(): start_node}
	
	# if plotter is not None:
	# 	utils.plotPoint(rand_node.getXYCoords(), plotter, radius=0.2, color='red')

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
			utils.plotPoint(step_node.getXYCoords(), plotter, radius=0.04, color='lime')
			cn_x, cn_y = closest_node.getXYCoords()
			sn_x, sn_y = step_node.getXYCoords()
			plotter.plot([cn_x, sn_x], [cn_y, sn_y], color='lime')
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
	
	prm.main()
	"""	rospy.loginfo("This start x pose: %s", start_pose_x)
		rospy.loginfo("This start y pose: %s", start_pose_y)
		rospy.loginfo("This goal x pose: %s", goal_pose_x)
		rospy.loginfo("This goal y pose: %s", goal_pose_y)"""
	#possition_listener()
	if (prm.start_pose_x == None) or (prm.start_pose_y == None):
		rospy.loginfo("Problem with start coordinates!")
		rospy.loginfo("Start x: '%s'" + " and " + "y: '%s' coordinates: ", str(prm.start_pose_x), str(prm.start_pose_y))
		sys.exit(0)
	elif  (prm.goal_pose_x == None) or (prm.goal_pose_y == None):
		rospy.loginfo("Problem with goal coordinates!")
		rospy.loginfo("Goal x: '%s'" + " and " + "y: '%s' coordinates: ", str(prm.goal_pose_x), str(prm.goal_pose_y))
		sys.exit(0)

	start_node = node_rrt.Node_rrt(current_coords=(prm.start_pose_x, prm.start_pose_y), parent_coords=None, distance=0)
	goal_node = node_rrt.Node_rrt(current_coords=(prm.goal_pose_x, prm.goal_pose_y), parent_coords=None, distance=0)

	start1 = start_node.getXYCoords()
	quat = [0,0,0,1]
	state_msg = ModelState()
	state_msg.model_name = 'iris'
	state_msg.pose.position.x = start1[0]
	state_msg.pose.position.y = start1[1]
	state_msg.pose.position.z = 0
	state_msg.pose.orientation.x = quat[0]
	state_msg.pose.orientation.y = quat[1]
	state_msg.pose.orientation.z = quat[2]
	state_msg.pose.orientation.w = quat[3]

	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	resp = set_state( state_msg )


	fig, ax = plt.subplots()
	ax.set_xlim(prm._lim_x, prm.lim_x)
	ax.set_ylim(prm._lim_y, prm.lim_y)
	fig.gca().set_aspect('equal', adjustable='box')
	utils.plotPoint(start_node.getXYCoords(), ax, radius=0.06, color='cyan') 	# start
	utils.plotPoint(goal_node.getXYCoords(), ax, radius=0.06, color='magenta') 	# end

	obs.generateMap(ax)

	plt.ion()

	rrt_path, _, itr = rrtPlannedPath(start_node, goal_node, robot_radius=DRONE_RADIUS, plotter=None, write=False)
	itr = 0
	# plt.savefig(('./frames/%04d.png' % (itr))); itr += 1
	# print(rrt_path)
	if rrt_path is not None:
		itr = utils.plotPath(rrt_path, plotter=ax, itr=itr, path_color='black'); itr += 1
	# plt.ioff()
	# plt.show()

	# Pruning
	path_co = np.array(utils.convertNodeList2CoordList(rrt_path))

	print(path_co.shape)
	rrt_prune_smooth_path_coords=path_pruning.prunedPath(path=path_co, radius=DRONE_RADIUS, clearance=(DRONE_RADIUS/2))
	rrt_prune_smooth_path_coords= np.array(rrt_prune_smooth_path_coords[::-1])
	# plt.plot(path_co[:,0],path_co[:,1],'green')
	# plt.plot(rrt_prune_smooth_path_coords[:,0],rrt_prune_smooth_path_coords[:,1],'cyan')
	# if path_co is not None:
	# 	utils.plotPath(path_co, plotter=ax, itr=itr, path_color='black')
	if rrt_prune_smooth_path_coords is not None:
		itr = utils.plotPath(rrt_prune_smooth_path_coords, plotter=ax, itr=itr, path_color='cyan'); itr += 1

	#rrt_smoothed_path_coors = utils.convertNodeList2CoordList(node_list=rrt_prune_smooth_path_coords)
	#smoothed_path_length = utils.path_length_meters(rrt_smoothed_path_length)
	smoothed_path_pength = utils.path_length_meters(rrt_prune_smooth_path_coords)
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

	plt.ioff()
	plt.show()

	plt.savefig(('./frames/%04d.png' % (itr))); itr += 1

	rrt_path_coords = utils.convertNodeList2CoordList(node_list=rrt_path)

	#np.save(file='rrt_path_nodes.npy', arr=rrt_path)
	#np.save(file='rrt_path_coords.npy', arr=rrt_path_coords)
	np.save(file='rrt_sim_smooth_path_coords.npy', arr=rrt_prune_smooth_path_coords)

if __name__ == '__main__':
	main()
