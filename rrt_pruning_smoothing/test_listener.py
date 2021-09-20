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

from geometry_msgs.msg import Point


from pickle import TRUE
import sys

start_pose_x = None
start_pose_y = None
goal_pose_x = None
goal_pose_y = None

def start_callback(data): 
    global start_pose_x
    global start_pose_y 
    start_pose_x = data.x
    start_pose_y = data.y
    rospy.loginfo("I heard start %s ", str(start_pose_x) + " and " + str(start_pose_y))

def goal_callback(data): 
    global goal_pose_x
    global goal_pose_y  
    goal_pose_x = data.x
    goal_pose_y = data.y
    rospy.loginfo("I heard goal %s ", str(goal_pose_x) + " and " + str(goal_pose_y))

def possition_listener():
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('possition_listener', anonymous=True)
        rospy.Subscriber('/start_point/position', Point, start_callback)
        rospy.Subscriber('/goal_point/position', Point, goal_callback)
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        rospy.sleep(2)

def output():
    print(start_pose_x)
    print(start_pose_y)

    print(goal_pose_x)
    print(goal_pose_y)


if __name__ == '__main__':
    possition_listener()
    if (start_pose_x == None) or (start_pose_y == None):
        rospy.loginfo("Problem with start coordinates!")
        rospy.loginfo("Start x: '%s'" + " and " + "y: '%s' coordinates: ", str(start_pose_x), str(start_pose_y))
        sys.exit(0)
    elif  (goal_pose_x == None) or (goal_pose_y == None):
        rospy.loginfo("Problem with goal coordinates!")
        rospy.loginfo("Goal x: '%s'" + " and " + "y: '%s' coordinates: ", str(goal_pose_x), str(goal_pose_y))
        sys.exit(0)
    #goal_listener()
    #if (goal_pose_x == None) or (goal_pose_y == None):
    #    rospy.loginfo("Problem with goal coordinates!")
    #    rospy.loginfo("Goal x: '%s'" + " and " + "y: '%s' coordinates: ", str(goal_pose_x), str(goal_pose_y))
    #    sys.exit(0)
    output()
