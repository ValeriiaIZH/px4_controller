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

def test_talker():
    rospy.init_node('test_talker')
    #obstacles_publisher = rospy.Publisher('/path/obstacles', Int64MultiArray, queue_size=10)
    number_obstacles_publisher = rospy.Publisher('/path/number_obstacles', Int64MultiArray, queue_size=10)
    lim_obstacles_publisher = rospy.Publisher('/path/lim_obstacles', Int64MultiArray, queue_size=10)
    path_planing_param_publisher = rospy.Publisher('/path/planing_param', Int64MultiArray, queue_size=10)
    start_point_publisher = rospy.Publisher('/start_point/position', Point, queue_size=10)
    goal_point_publisher = rospy.Publisher('/goal_point/position', Point, queue_size=10)

    #max_iter_publisher = rospy.Publisher('/path/max_iter', Int64MultiArray, queue_size=10)
    #step_size_publisher = rospy.Publisher('/path/step_size', Int64MultiArray, queue_size=10)
    #goal_reach_thresh_publisher = rospy.Publisher('/path/goal_reach_thresh', Int64MultiArray, queue_size=10)
    #drone_radius_publisher = rospy.Publisher('/path/drone_radius', Int64MultiArray, queue_size=10)

    #obstacles = Int64MultiArray()
    number_obstacles = Int64MultiArray()
    lim_obstacles = Int64MultiArray()
    path_planing_param = Int64MultiArray()
    start_point = Point()
    goal_point = Point()

    number_obstacles = 0
    start_point.x = -5001.787385427393
    start_point.y = 9526.705177228898
    goal_point.x = -2833
    goal_point.y = 4969

    _xlim = -6000
    xlim = -1000
    _ylim = 4000
    ylim =  10000

    max_iter = 50000
    step_size = 400
    goal_reach_thresh = 1000
    drone_radius = 2

    lim_obstacles.data = [_xlim, xlim, _ylim, ylim]
    path_planing_param.data = [max_iter, step_size, goal_reach_thresh, drone_radius]
    
    #max_iter = Int64MultiArray()
    #step_size = Int64MultiArray()
    #goal_reach_thresh = Int64MultiArray
    #drone_radius = Int64MultiArray()

    #rate = rospy.Rate(10) # 10hz

    #while not rospy.is_shutdown():
        #connections = goal_point.publish.get_num_connections()
        #rospy.loginfo('Connections: %d', connections)
        #if connections > 0:
    lim_obstacles_publisher.publish(lim_obstacles)
    path_planing_param_publisher.publish(path_planing_param)
    goal_point_publisher.publish(goal_point)
    start_point_publisher.publish(start_point)
            #rospy.loginfo('Published')
            #break
        #rate.sleep()

if __name__ == '__main__':
    try:
        
        from time import sleep
        print 'Sleeping 10 seconds to publish'
        sleep(10)
        print 'Sleep finished.'

        test_talker()

    except rospy.ROSInterruptException:
        pass
