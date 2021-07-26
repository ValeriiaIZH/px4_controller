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
import rospy
from rospy.exceptions import ROSInitException
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Point as P
from pickle import TRUE

#obsticals
pose_x = None
pose_y = None
pose_r = None

#x-y lim
_lim_x = -5
lim_x = 5
_lim_y = -5
lim_y = 5

start_pose_x = None
start_pose_y = None
goal_pose_x = None
goal_pose_y = None

number_obstacles = 1
possition = []

check_true = True

global c_max_iter
global c_step_size
global c_goal_reach_thresh
global c_drone_radius

def path_planing_param_callback(data):
	global c_max_iter
	global c_step_size
	global c_goal_reach_thresh
	global c_drone_radius
	c_max_iter = data.data[0]
	c_step_size = data.data[1]
	c_goal_reach_thresh = data.data[2]
	c_drone_radius = data.data[3]
def number_obstacles_callback(data):
	global number_obstacles
	number_obstacles = data.data[0]
	rospy.loginfo("This is number: %s", str(number_obstacles))
def obstacles_pos_callback(data):
	global pose_x
	global pose_y
	global pose_r
	pose_x = data.data[0]
	pose_y = data.data[1]
	pose_r = data.data[2]
	rospy.loginfo("This is possition: %s", str(pose_x) + " " + str(pose_y) + " " + str(pose_r))
	array_1 = [pose_x, pose_y, pose_r]
	possition.append(array_1)
	print(possition)
def lim_obstacles_callback(data):
	global _lim_x
	global lim_x
	global _lim_y
	global lim_y

	_lim_x = data.data[0]
	lim_x = data.data[1]
	_lim_y = data.data[2]
	lim_y = data.data[3]
	rospy.loginfo("This is limit: -x:%s, x:%s, -y:%s, y:%s", str(_lim_x), str(lim_x), str(_lim_y), str(lim_y))
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

def main():
	global check_true
	# spin() simply keeps python from exiting until this node is stopped
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('main', anonymous=True)
	rospy.Subscriber('/start_point/position', P, start_pos_callback)
	rospy.Subscriber('/goal_point/position', P, goal_pos_callback)
	rospy.Subscriber('/path/number_obstacles', Int64MultiArray, number_obstacles_callback)
	rospy.Subscriber('/path/obstacles', Int64MultiArray, obstacles_pos_callback)
	rospy.Subscriber('/path/lim_obstacles', Int64MultiArray, lim_obstacles_callback)
	rospy.Subscriber('/path/planing_param', Int64MultiArray, path_planing_param_callback)
	#rospy.sleep(10)
	r = rospy.Rate(10)
	while check_true == True:
		if (len(possition) >= number_obstacles):
			check_true = False
			print(check_true)
			print(possition)
		else: 
			r.sleep()

if __name__ == '__main__':
	main()