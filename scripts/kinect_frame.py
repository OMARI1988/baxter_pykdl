#!/usr/bin/python

import rospy
import tf
import numpy as np
import cv2

global q,xyz,q_calib1,xyz_calib1,rpy
xyz = [0.0, 0.0, 0.0]
q = [0.0, 0.0, 0.0, 1.0]
rpy = [0,0,0]

xyz_calib1 = [0.29800000000000004, -0.07499999999999996, 0.2350000000000001]
q_calib1 = [0.64694529, -0.63834098,  0.29837979, -0.2914654]
rpy_calib = [2293, 3592, 2704]      # the values you should use

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    #
    def nothing(x):
        global q,xyz,rpy
        # get current positions of four trackbars
        xyz[0] = cv2.getTrackbarPos('x','image')/1000.0-1
        xyz[1] = cv2.getTrackbarPos('y','image')/1000.0-1
        xyz[2] = cv2.getTrackbarPos('z','image')/1000.0-1
        r = cv2.getTrackbarPos('Roll','image')
        p = cv2.getTrackbarPos('Pitch','image')
        y = cv2.getTrackbarPos('Yaw','image')
        rpy = [r,p,y]
        q = tf.transformations.quaternion_from_euler( r*np.pi/1800, p*np.pi/1800, y*np.pi/1800)
        # q = tuple(qq)

    # Create a black image, a window
    img = np.zeros((300,512,3), np.uint8)
    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('x','image',0,3000,nothing)
    cv2.createTrackbar('y','image',0,3000,nothing)
    cv2.createTrackbar('z','image',0,3000,nothing)
    cv2.createTrackbar('Roll','image',0,3600,nothing)
    cv2.createTrackbar('Pitch','image',0,3600,nothing)
    cv2.createTrackbar('Yaw','image',0,3600,nothing)

    while not rospy.is_shutdown():
        # cv2.imshow('image',img)
        # k = cv2.waitKey(1) & 0xFF
        # print 'xyz:',xyz
        # print 'rpy:',rpy
        # print 'q:',q
        # print '------------------'

        # br2.sendTransform((xyz[0], xyz[1], xyz[2]),
        #                  (q[0],  q[1],  q[2],  q[3]),
        #                  rospy.Time.now(),
        #                  "kinect2_1_ir_optical_frame",
        #                  "torso")

        br1.sendTransform((xyz_calib1[0], xyz_calib1[1], xyz_calib1[2]),
                         (q_calib1[0],  q_calib1[1],  q_calib1[2],  q_calib1[3]),
                         rospy.Time.now(),
                         "kinect2_1_rgb_optical_frame",
                         "torso")

        br2.sendTransform((xyz_calib1[0], xyz_calib1[1], xyz_calib1[2]),
                         (q_calib1[0],  q_calib1[1],  q_calib1[2],  q_calib1[3]),
                         rospy.Time.now(),
                         "kinect2_1_ir_optical_frame",
                         "torso")
        rate.sleep()
