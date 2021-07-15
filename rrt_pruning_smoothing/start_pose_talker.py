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

from re import T
import rospy

from geometry_msgs.msg import Point


def start_point():
    check_input = True
    rospy.init_node('start_point')
    start_point_publisher = rospy.Publisher('/start_point/position', Point, queue_size=10)
    start_point = Point()
    rate = rospy.Rate(5) # 10hz

    while not rospy.is_shutdown(): #check_input == True:

        start_point.x = -4.0
        start_point.y = -4.0

        start_point_publisher.publish(start_point)

        rate.sleep()

if __name__ == '__main__':
    try:
        start_point()
    except rospy.ROSInterruptException:
        pass
"""
def talker():
    global x
    global y
    x = 0
    y = 0
    check_input = True
    pub = rospy.Publisher('chatter', PoseStamped, queue_size=2)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while   check_input == True:
        start_str_x = "Input start point x: "
        start_str_y = "Input start point y: "
        rospy.loginfo(start_str_x)
        x = raw_input()
        rospy.loginfo(start_str_y)
        y = raw_input()
        
        pub.publish(float(x))
        #pub.publish(float(y))

        rate.sleep()
        if float (x) != None:
            check_input = False
"""
"""
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
"""