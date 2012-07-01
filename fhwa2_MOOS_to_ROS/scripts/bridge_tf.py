#!/usr/bin/env python

"""
This file contains tf's used by bridge.py
Created 5/30/12 by Robert Cofield
"""

import roslib
roslib.load_manifest('fhwa2_MOOS_to_ROS')
import tf

def cameraFollow_tf(self, odom_msg):
        # msg = self.odom_msgs[time]
        msg = odom_msg
        br = tf.TransformBroadcaster()

        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), # send zjj in case
                         (0, 0, 0, 1), # this is a unit quaternion
                         msg.header.stamp,
                         "base_footprint", # child frame
                         "odom") #parent frame