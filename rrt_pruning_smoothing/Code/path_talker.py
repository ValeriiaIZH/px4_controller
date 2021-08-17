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

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
import sys
import main_no_sim as mns
import time
import json

# reed from file
with open('path.txt') as f:
        rrt_path_coords = json.loads(f.read())

def path_message(PATH):
	path = Path()
	path.header = Header(frame_id="map", stamp=rospy.Time.now())
	for pt in PATH:
		pose = PoseStamped()
		pose.header = path.header
		pose.pose.position = Point(pt[0], pt[1], 10)
		#pose.pose.orientation = Quaternion(0,0,0,1)

		path.poses.append(pose)
	return path


def path_rrt_talker():

	rospy.init_node('path_rrt_talker')
	rospy.loginfo("Path publisher")
	rate = rospy.Rate(1)
    

	# start publisher
	path_rrt_publisher = rospy.Publisher('/path_rrt/path', Path, queue_size=10)

	my_path = path_message(rrt_path_coords)

	while not rospy.is_shutdown():
		path_rrt_publisher.publish(my_path)
        rate.sleep(1)

if __name__ == '__main__':	
	
    path_rrt_talker()

