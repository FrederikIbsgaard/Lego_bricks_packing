#!/usr/bin/env python2
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

# Simple talker demo that listens to std_msgs/Strings published
# to the 'chatter' topic

import rospy
import cv2
from std_msgs.msg import String
import utilities as utilities
import os


# Path to configuration files
utilities.setPathsToConfigurationFiles("~/catkin_ws/src/Lego_bricks_packing/src/vision/conf/Cropping_values.json",
                                       "~/catkin_ws/src/Lego_bricks_packing/src/vision/conf/HSV_thresholds.json",
                                       "~/catkin_ws/src/Lego_bricks_packing/src/vision/conf/RGB_thresholds.json")


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    # Adquire image
    state, image = utilities.loadImage("lego")

    rate = rospy.Rate(0.5) # 10hz
    # Check that the input image exist
    if state == utilities.SUCCESS:
        output = utilities.cropImage(image)  
        color = utilities.getColor(output,utilities.HSV)
        print("INFO: (HSV) predominant color matches: " + color)
        color = utilities.getColor(output,utilities.RGB)
        print("INFO: (RGB) predominant color matches: " + color)  

        # revise if this is the proper way
        pub = rospy.Publisher('vision_response', String, queue_size=10)
        pub.publish(color)

def listener():

    # Initialize node
    pub = rospy.Publisher('vision_respone', String, queue_size=10)
    rospy.init_node('brick_color_detector', anonymous=True)
    rospy.Subscriber('vision_request', String, callback)      

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
