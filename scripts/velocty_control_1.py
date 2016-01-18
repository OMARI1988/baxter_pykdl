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

from std_msgs.msg import (
    UInt16,
)

import baxter_interface
from baxter_interface import CHECK_VERSION


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()
        self.q_dot =[.0,.0,.0,.0,.0,.0,.0]
        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

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

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

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
        # self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_cmd(joint_names, q_dot):
            return dict([(joint, q_dot[i])
                         for i, joint in enumerate(joint_names)])

        for i in range(3):
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._left_joint_names, self.q_dot)
            self._left_arm.set_joint_velocities(cmd)
            # cmd = make_cmd(self._right_joint_names, elapsed)
            # self._right_arm.set_joint_velocities(cmd)
            rate.sleep()

def main():
    rospy.init_node('baxter_velocity_control')
    print '*** Baxter PyKDL Kinematics ***\n'
    kin = baxter_kinematics('left')

    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()
    # # FK Position
    # print '\n*** Baxter Position FK ***\n'
    # print kin.forward_position_kinematics()
    # # FK Velocity
    # # print '\n*** Baxter Velocity FK ***\n'
    # # kin.forward_velocity_kinematics()
    # # IK
    # print '\n*** Baxter Position IK ***\n'
    # pos = [0.582583, -0.180819, 0.216003]
    # rot = [0.03085, 0.9945, 0.0561, 0.0829]
    # print kin.inverse_kinematics(pos)  # position, don't care orientation
    # print '\n*** Baxter Pose IK ***\n'
    # print kin.inverse_kinematics(pos, rot)  # position & orientation
    # # Jacobian
    # print '\n*** Baxter Jacobian ***\n'
    # print kin.jacobian()
    # # Jacobian Transpose
    # print '\n*** Baxter Jacobian Tranpose***\n'
    # print kin.jacobian_transpose()
    # # Jacobian Pseudo-Inverse (Moore-Penrose)
    # print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    # print kin.jacobian_pseudo_inverse()
    # # Joint space mass matrix
    # print '\n*** Baxter Joint Inertia ***\n'
    # print kin.inertia()
    # # Cartesian space mass matrix
    # print '\n*** Baxter Cartesian Inertia ***\n'
    # print kin.cart_inertia()

    wobbler = Wobbler()

    for i in range(300):
        print i
        J = kin.jacobian_pseudo_inverse()
        A = np.matrix(J)
        twist = np.array([[-.06],[.0],[.0],[0],[0],[0]])
        q_dot = A*twist
        q_dot_list = [i[0] for i in q_dot.tolist()]
        print q_dot_list
        wobbler.q_dot = q_dot_list
        wobbler.wobble()

    wobbler.q_dot = [.0,.0,.0,.0,.0,.0,.0]
    wobbler.wobble()

    rospy.on_shutdown(wobbler.clean_shutdown)

    print("Done.")

if __name__ == '__main__':
    main()
