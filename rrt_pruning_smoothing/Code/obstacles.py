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

from operator import pos
import sys
import json
import os

from numpy.core.defchararray import array
from six.moves import xrange
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import matplotlib.pyplot as plt
import numpy as np
import parameter_listener as prm

def possition_obstacles():
	for i in range(prm.number_obstacles):
		print(i)
		print(prm.possition)
		array_1 = [prm.pose_x, prm.pose_y, prm.pose_r]
		prm.possition.append(array_1)
		print(prm.possition)

def withinObstacleSpace(point, radius, clearance):
	 
	x = point[0]
	y = point[1]

	flag = False
	point = Point(x, y)
	# TODO: Add the ability to read multiple obstacles
	# creating rectangles from file
	json_data = []
	x_arr_obs = []
	y_arr_obs = []
	
	if os.path.exists('Obs.json'):	
		with open('Obs.json', 'r') as f:
			json_data = json.load(f)
			points_arr = json_data.get('points')
		for i in range(len(points_arr)):
			points_arr_ = points_arr[i]
			x = points_arr_[0]
			y = points_arr_[1]
			x_arr_obs.append(x)
			y_arr_obs.append(y)
		rectangle_1 = Polygon(zip(x_arr_obs,y_arr_obs))
		if point.distance(rectangle_1) <= radius + clearance:
			flag = True

	for i in xrange(len(prm.possition)):
		print(prm.possition)
		number_circle = i
		p_ = i
		number_circle = plt.Circle((prm.possition[i][0], prm.possition[i][1]), prm.possition[i][2], color='b')
		
		# creating circles
		p_ = Point((prm.possition[i][0], prm.possition[i][1]))
		number_circle = p_.buffer(prm.possition[i][2])

		if point.distance(number_circle) <= radius + clearance:
			flag = True
	return flag


def withinObstacleSpaceFake((x, y), radius, clearance):
	"""
	Always returns False. Only used for debugging purposes.

	:param      (x,y)):     Point to be checked
	:type       (x,y)):     tuple of (x,y) coordinates
	:param      radius:     The robot radius
	:type       radius:     float
	:param      clearance:  The robot clearance from objects
	:type       clearance:  float
	:param      plotter:    The plotter for visualization
	:type       plotter:    matplotlib.pyplot

	:returns:   Always False
	:rtype:     boolean
	"""
	return False


def generateMap(plotter=plt):
	"""
	Genarting the map for the obstacle space

	:param      plotter:  The plotter
	:type       plotter:  matplotlib.pyplot
	"""
	#obstacles_listener()
	#possition = (pose_x, pose_y, pose_r) 
	
	json_data = []
	x_arr_obs = []
	y_arr_obs = []
	if os.path.exists('Obs.json'):	
		with open('Obs.json', 'r') as f:
			json_data = json.load(f)
			points_arr = json_data.get('points')
		for i in range(len(points_arr)):
			points_arr_ = points_arr[i]
			x = points_arr_[0]
			y = points_arr_[1]
			x_arr_obs.append(x)
			y_arr_obs.append(y)
		rectangle_1 = plt.Polygon(zip(x_arr_obs,y_arr_obs))
		plotter.add_line(rectangle_1)
	else:
		print('####!!!!There is no file with obstacles!!!!####')

	for i in xrange(len(prm.possition)):
		print(prm.possition)

		number_circle = i
		print(i)
		number_circle = plt.Circle((prm.possition[i][0], prm.possition[i][1]), prm.possition[i][2], color='b')
		#circle_2 = plt.Circle((-2, -3), 1, color='b')
		#circle_3 = plt.Circle((2, -3), 1, color='b')
		#circle_4 = plt.Circle((2, 3), 1, color='b')
	
		plotter.add_artist(number_circle)
		#plotter.add_artist(circle_2)
		#plotter.add_artist(circle_3)
		#plotter.add_artist(circle_4)
	
def testMain():
	# x = float(sys.argv[1])
	# y = float(sys.argv[2])
	
	fig, ax = plt.subplots()
	ax.set(xlim=(prm._lim_x, prm.lim_x), ylim=(prm._lim_y, prm.lim_y))
	ax.set_aspect('equal')
	# ax.plot([x], [y], color="black", marker="+", markersize=3)

	# print(withinObstacleSpace((x, y), 0.105, 0.2, plotter=ax))
	
	generateMap(ax)

	plt.show()

if __name__ == '__main__':
	prm.main()
	#obstacles_listener()
	#possition_obstacles()
	testMain()
	
