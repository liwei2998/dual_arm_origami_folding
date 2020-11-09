#!/usr/bin/env python  
import roslib

import numpy
import rospy
import tf
from math import pi

if __name__ == '__main__':
    rospy.init_node('frame_transform')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0, axes='sxyz')

    while not rospy.is_shutdown():
        br.sendTransform((0.09874199999970079, 0.11680999999968875, 0.061000000001079635),
                 (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                 rospy.Time.now(),
                 "soft_gripper_kong","kong_ee_link")

        br.sendTransform((0.09874199999970079, 0.11680999999968875, 0.061000000001079635),
                 (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                 rospy.Time.now(),
                 "soft_gripper_hong","hong_ee_link")

        br.sendTransform((0.140,0.04,0.06),
                 (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                 rospy.Time.now(),
                 "pinch_tip_hong","hong_ee_link")
        rate.sleep()
