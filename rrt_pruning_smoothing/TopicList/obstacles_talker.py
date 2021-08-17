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
from re import T, X
import rospy

from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Point
import math

def obstacles_talker():
    rospy.init_node('obstacles_talker')
    obstacles_publisher = rospy.Publisher('/path/obstacles', Int64MultiArray, queue_size=10)
    number_obstacles_publisher = rospy.Publisher('/path/number_obstacles', Int64MultiArray, queue_size=10)
    
    lim_obstacles_publisher = rospy.Publisher('/path/lim_obstacles', Int64MultiArray, queue_size=10)
    path_planing_param_publisher = rospy.Publisher('/path/planing_param', Int64MultiArray, queue_size=10)

    start_point_publisher = rospy.Publisher('/start_point/position', Point, queue_size=10)
    goal_point_publisher = rospy.Publisher('/goal_point/position', Point, queue_size=10)
    #max_iter_publisher = rospy.Publisher('/path/max_iter', Int64MultiArray, queue_size=10)
    #step_size_publisher = rospy.Publisher('/path/step_size', Int64MultiArray, queue_size=10)
    #goal_reach_thresh_publisher = rospy.Publisher('/path/goal_reach_thresh', Int64MultiArray, queue_size=10)
    #drone_radius_publisher = rospy.Publisher('/path/drone_radius', Int64MultiArray, queue_size=10)

    obstacles = Int64MultiArray()
    number_obstacles = Int64MultiArray()
    
    lim_obstacles = Int64MultiArray()
    path_planing_param = Int64MultiArray()

    start_point = Point()
    goal_point = Point()
    #max_iter = Int64MultiArray()
    #step_size = Int64MultiArray()
    #goal_reach_thresh = Int64MultiArray
    #drone_radius = Int64MultiArray()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown(): 
        
        start_point.x = input("Input start x point: ")
        start_point.y = input("Input start y point: ")
        goal_point.x = input("Input goal x point: ")
        goal_point.y = input("Input goal y point: ")

        number = input("Input number obstacles: ")
        print("Input plot limit")
        _xlim = input("Input -x: ")
        xlim = input("Input +x: ")
        _ylim = input("Input -y: ")
        ylim = input("Input +y: ")

        number_obstacles.data = [number] 
        number_obstacles_publisher.publish(number_obstacles)

        lim_obstacles.data = [_xlim, xlim, _ylim, ylim]
        lim_obstacles_publisher.publish(lim_obstacles)


        s_aria = (abs(_xlim)+abs(xlim))*(abs(_ylim)+abs(ylim))

        max_iter = 50000
        step_size = 1
        goal_reach_thresh = 2
        #goal_reach_thresh = math.sqrt(s_aria)*0.25
        drone_radius = 1

        path_planing_param.data = [max_iter, step_size, goal_reach_thresh, drone_radius]
        path_planing_param_publisher.publish(path_planing_param)

        start_point_publisher.publish(start_point)
        goal_point_publisher.publish(goal_point)

        for i in range(number):
            
            x = input("Input x coor: ")
            y = input("Input y coor: ")
            z = input("Input radius coor: ")

            obstacles.data = [x, y, z]
            obstacles_publisher.publish(obstacles)

            rate.sleep()


if __name__ == '__main__':
    try:
        obstacles_talker()
    except rospy.ROSInterruptException:
        pass
