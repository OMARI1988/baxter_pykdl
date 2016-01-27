#!/usr/bin/python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.19509032,  0.0 , 0.98078528),
                         rospy.Time.now(),
                         "kinect2_rgb_optical_frame",
                         "torso")
        rate.sleep()
