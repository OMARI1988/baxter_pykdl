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

from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy,Image
from baxter_pykdl.msg import joy_stick_commands
import baxter_interface
from baxter_interface import CHECK_VERSION
import cv2
from cv_bridge import CvBridge


class Camera():

    def __init__(self):
        rospy.Subscriber("/cameras/right_hand_camera/image", Image, self._right_camera)
        self._br = CvBridge()	# Create a black image, a window
        self.count = 0

    def _right_camera(self,imgmsg):
        img = self._br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        print img.shape
    	cv2.imshow('right',img)
    	k = cv2.waitKey(1) & 0xff
        cv2.imwrite('/home/omari/Datasets/test/image_'+str(self.count))

class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate          = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm          = baxter_interface.limb.Limb("left")
        self._right_arm         = baxter_interface.limb.Limb("right")
        self._left_joint_names  = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()
        self._left_grip = baxter_interface.Gripper('left', CHECK_VERSION)
        self._right_grip = baxter_interface.Gripper('right', CHECK_VERSION)
        # calibrate
        self._left_grip.calibrate()
        self._right_grip.calibrate()
        # self._left_grip_state   = 'open'
        # self._right_grip_state  = 'open'
        # control parameters
        self._rate              = 500.0  # Hz
        self._max_speed         = .05
        self._arm               = 'none'
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)
        rospy.Subscriber("joy", Joy, self._joystick_read)
        # Dynamics
        self.q_dot              = [.0,.0,.0,.0,.0,.0,.0]
        self.twist              = np.array([[.0],[.0],[.0],[.0],[.0],[.0]])
        print '*** Baxter PyKDL Kinematics ***\n'
        self.kin = {}
        self.kin['left'] = baxter_kinematics('left')
        self.kin['right'] = baxter_kinematics('right')
        self.pub_joy = rospy.Publisher('joy_commands', joy_stick_commands, queue_size=10)


    def _joystick_read(self,data):
        # reset arm positions
        if data.buttons[6]:
            self.set_neutral()
        # which arm to control
        if data.buttons[4]:     self._arm = 'left'
        if data.buttons[5]:     self._arm = 'right'
        print 'buttons',data.buttons
        # grippers
        if data.buttons[0]:
            if self._arm == 'left' : self._left_grip.close()
            if self._arm == 'right': self._right_grip.close()
        if data.buttons[1]:
            if self._arm == 'left' : self._left_grip.open()
            if self._arm == 'right': self._right_grip.open()
        # x_axis speed
        self.twist[0][0] = data.axes[1]*self._max_speed + data.axes[7]*self._max_speed*.9
        # y_axis speed
        self.twist[1][0] = data.axes[0]*self._max_speed + data.axes[6]*self._max_speed
        # z_axis speed
        self.twist[2][0] = ((data.axes[2]-1)-(data.axes[5]-1))*self._max_speed/2.0
        # # rot x_axis speed
        # self.twist[3][0] = -data.axes[3]*self._max_speed
        # # rot y_axis speed
        # self.twist[4][0] = data.axes[4]*self._max_speed

        if self._arm in self.kin:
            J = np.matrix(self.kin[self._arm].jacobian_pseudo_inverse())
            # A = np.matrix(J)
            q_dot = J*self.twist
            self.q_dot = [i[0] for i in q_dot.tolist()]
            self.q_dot[-1] += data.buttons[2]*.5 - data.buttons[3]*.5
            self.q_dot[-2] += data.axes[4]*.5
            self.q_dot[-3] -= data.axes[3]*.5
        T = [self.twist[0][0], self.twist[1][0], self.twist[2][0], 'not_used', 'not_used', 'not_used']
        self.pub_joy.publish(str(data.buttons),str(data.axes),str(T),str(self.q_dot),self._arm)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def make_cmd(self,joint_names, q_dot):
        return dict([(joint, q_dot[i])
                     for i, joint in enumerate(joint_names)])

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        self._left_arm.set_joint_position_speed(.8)
        self._right_arm.set_joint_position_speed(.8)
        print("Moving to original pose...")
        # self._left_arm.move_to_neutral()
        p = [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50]
        cmd = self.make_cmd(self._left_joint_names, p)
        self._left_arm.move_to_joint_positions(cmd,15,.03)
        p = [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]
        cmd = self.make_cmd(self._right_joint_names, p)
        self._right_arm.move_to_joint_positions(cmd,15,.03)

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        # self.set_neutral()
        # if not self._init_state:
        #     print("Disabling robot...")
        #     self._rs.disable()
        return True

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()


        while not rospy.is_shutdown():
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            if self._arm == 'left':
                cmd = self.make_cmd(self._left_joint_names, self.q_dot)
                self._left_arm.set_joint_velocities(cmd)
            elif self._arm == 'right':
                cmd = self.make_cmd(self._right_joint_names, self.q_dot)
                self._right_arm.set_joint_velocities(cmd)
            rate.sleep()

def main():
    rospy.init_node('baxter_velocity_control')

    # # FK Position
    # print '\n*** Baxter Position FK ***\n'
    # print kin.forward_position_kinematics()

    # camera = Camera()
    wobbler = Wobbler()


    # for i in range(300):
    #     print i
    #     J = kin.jacobian_pseudo_inverse()
    #     A = np.matrix(J)
    #     twist = np.array([[.0],[.0],[.0],[.0],[.0],[.0]])
    #     q_dot = A*twist
    #     q_dot_list = [i[0] for i in q_dot.tolist()]
    #     # print q_dot_list
    #     wobbler.q_dot = q_dot_list
    #     wobbler.wobble()

    wobbler.q_dot = [.0,.0,.0,.0,.0,.0,.0]
    wobbler.wobble()
    # rospy.spin()
    rospy.on_shutdown(wobbler.clean_shutdown)

    print("Done.")

if __name__ == '__main__':
    main()
