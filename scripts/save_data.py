#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO Enp.asarray(a)VENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import numpy as np
from baxter_pykdl import baxter_kinematics
import random
import math
import pcl

from std_msgs.msg import (
    UInt16,
)

from sensor_msgs.msg import Joy,Image
import baxter_interface
from baxter_interface import CHECK_VERSION
import cv2
from cv_bridge import CvBridge
from baxter_core_msgs.msg import EndpointState
import time

class Robot():

    def __init__(self):
        self.right = 1
        self.left = 1
        self.R_count = 1
        self.L_count = 1
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self._right_endpoint)
        rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self._left_endpoint)

    def _right_endpoint(self,msg):
        if self.right:
            if self.R_count < 10:
                f = open('/home/omari/Datasets/test3/Endpoint_right_0000'+str(self.R_count)+'.txt','w')
            elif self.R_count < 100:
                f = open('/home/omari/Datasets/test3/Endpoint_right_000'+str(self.R_count)+'.txt','w')
            elif self.R_count < 1000:
                f = open('/home/omari/Datasets/test3/Endpoint_right_00'+str(self.R_count)+'.txt','w')
            elif self.R_count < 10000:
                f = open('/home/omari/Datasets/test3/Endpoint_right_0'+str(self.R_count)+'.txt','w')
            self.R_count += 1
            f.write('position\n')
            f.write('x:'+str(msg.pose.position.x)+'\n')
            f.write('y:'+str(msg.pose.position.y)+'\n')
            f.write('z:'+str(msg.pose.position.z)+'\n')
            f.write('orientation\n')
            f.write('x:'+str(msg.pose.orientation.x)+'\n')
            f.write('y:'+str(msg.pose.orientation.y)+'\n')
            f.write('z:'+str(msg.pose.orientation.z)+'\n')
            f.write('w:'+str(msg.pose.orientation.w)+'\n')
            f.close()
            self.right = 0

    def _left_endpoint(self,msg):
        if self.left:
            if self.L_count < 10:
                f = open('/home/omari/Datasets/test3/Endpoint_left_0000'+str(self.L_count)+'.txt','w')
            elif self.L_count < 100:
                f = open('/home/omari/Datasets/test3/Endpoint_left_000'+str(self.L_count)+'.txt','w')
            elif self.L_count < 1000:
                f = open('/home/omari/Datasets/test3/Endpoint_left_00'+str(self.L_count)+'.txt','w')
            elif self.L_count < 10000:
                f = open('/home/omari/Datasets/test3/Endpoint_left_0'+str(self.L_count)+'.txt','w')
            self.L_count += 1
            f.write('position\n')
            f.write('x:'+str(msg.pose.position.x)+'\n')
            f.write('y:'+str(msg.pose.position.y)+'\n')
            f.write('z:'+str(msg.pose.position.z)+'\n')
            f.write('orientation\n')
            f.write('x:'+str(msg.pose.orientation.x)+'\n')
            f.write('y:'+str(msg.pose.orientation.y)+'\n')
            f.write('z:'+str(msg.pose.orientation.z)+'\n')
            f.write('w:'+str(msg.pose.orientation.w)+'\n')
            f.close()
            self.left = 0

class Camera():

    def __init__(self):
        self._br = CvBridge()	# Create a black image, a window
        self.count1 = 1
        self.count2 = 1
        self.count3 = 1
        self.img1 = 1
        self.img2 = 1
        self.img3 = 1
        rospy.Subscriber("/cameras/right_hand_camera/image", Image, self._right_camera)
        rospy.Subscriber("/cameras/left_hand_camera/image", Image, self._left_camera)
        rospy.Subscriber("/kinect2/qhd/image_color", Image, self._kinect)

    def _right_camera(self,imgmsg):
        img = self._br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        # print img.shape
    	cv2.imshow('right',img)
    	k = cv2.waitKey(1) & 0xff
        if self.img1:
            if self.count1 < 10:
                cv2.imwrite('/home/omari/Datasets/test3/right_0000'+str(self.count1)+'.png',img)
            elif self.count1 < 100:
                cv2.imwrite('/home/omari/Datasets/test3/right_000'+str(self.count1)+'.png',img)
            elif self.count1 < 1000:
                cv2.imwrite('/home/omari/Datasets/test3/right_00'+str(self.count1)+'.png',img)
            elif self.count1 < 10000:
                cv2.imwrite('/home/omari/Datasets/test3/right_0'+str(self.count1)+'.png',img)
            self.count1 += 1
            self.img1 = 0

    def _left_camera(self,imgmsg):
        img = self._br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        # print img.shape
    	cv2.imshow('left',img)
    	k = cv2.waitKey(1) & 0xff
        if self.img2:
            if self.count2 < 10:
                cv2.imwrite('/home/omari/Datasets/test3/left_0000'+str(self.count2)+'.png',img)
            elif self.count2 < 100:
                cv2.imwrite('/home/omari/Datasets/test3/left_000'+str(self.count2)+'.png',img)
            elif self.count2 < 1000:
                cv2.imwrite('/home/omari/Datasets/test3/left_00'+str(self.count2)+'.png',img)
            elif self.count2 < 10000:
                cv2.imwrite('/home/omari/Datasets/test3/left_0'+str(self.count2)+'.png',img)
            self.count2 += 1
            self.img2 = 0

    def _kinect(self,imgmsg):
        img = self._br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        # print img.shape
    	cv2.imshow('kinect',img)
    	k = cv2.waitKey(1) & 0xff
        if self.img3:
            if self.count3 < 10:
                cv2.imwrite('/home/omari/Datasets/test3/kinect_0000'+str(self.count3)+'.png',img)
            elif self.count3 < 100:
                cv2.imwrite('/home/omari/Datasets/test3/kinect_000'+str(self.count3)+'.png',img)
            elif self.count3 < 1000:
                cv2.imwrite('/home/omari/Datasets/test3/kniect_00'+str(self.count3)+'.png',img)
            elif self.count3 < 10000:
                cv2.imwrite('/home/omari/Datasets/test3/kinect_0'+str(self.count3)+'.png',img)
            self.count3 += 1
            self.img3 = 0

def main():
    rospy.init_node('baxter_save_data')
    robot = Robot()
    camera = Camera()
    while not rospy.is_shutdown():
        time.sleep(.5)
        camera.img1=1
        camera.img2=1
        camera.img3=1

        robot.right=1
        robot.left=1

    # rospy.on_shutdown()

    print("Done.")

if __name__ == '__main__':
    main()
