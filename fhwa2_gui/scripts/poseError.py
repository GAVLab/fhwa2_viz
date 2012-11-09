#!/usr/bin/env python
import sys, os
import roslib
roslib.load_manifest('fhwa2_gui')
import rospy
from fhwa2_gui.msg import PoseError
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from math import sqrt
from pprint import pprint as pp


class ErrorNode(object):
    """Calculates the position error between 2 given pose topics
    Input poses are in UTM
    Output magnitude is in meters, using predefined message format

    Only update the value error when target pose is updated. Always use the 
    reference pose stored.
    """
    def __init__(self):
        super(ErrorNode, self).__init__()
        self.DEBUG = bool(rospy.get_param('~DEBUG', 'False'))
        # set topics
        self.ref_topic = rospy.get_param('~ref_topic')
        self.tgt_topic = rospy.get_param('~tgt_topic')
        # set pubs/subs
        self.pub_topic = rospy.get_param('~pub_topic')
        self.pub = rospy.Publisher(self.pub_topic, PoseError)
        self.pub_test = rospy.Publisher('/test_error', Float64)
        self.ref_sub = rospy.Subscriber(self.ref_topic, Odometry, self.onRefUpdate)
        self.tgt_sub = rospy.Subscriber(self.tgt_topic, Odometry, self.onTgtUpdate)
        # Data holders
        self.ref_pose = Odometry()
        self.tgt_pose = Odometry()
        self.output = PoseError()
        if self.DEBUG: print 'ErrorNode output format:', pp(self.output)


    def onRefUpdate(self, msg):
        if self.DEBUG: print('In ErrorNode::onRefUpdate')
        self.ref_pose = msg


    def onTgtUpdate(self, msg):
        if self.DEBUG: print('In ErrorNode::onRefUpdate')
        self.tgt_pose = msg
        self.crunch()
        self.spit()


    def crunch(self):
        if self.DEBUG: print('In ErrorNode::onRefUpdate')
        x1 = self.ref_pose.pose.pose.position.x
        x2 = self.tgt_pose.pose.pose.position.x
        y1 = self.ref_pose.pose.pose.position.y
        y2 = self.tgt_pose.pose.pose.position.y
        mag = sqrt((x2-x1)**2 + (y2-y1)**2)
        self.output.mag_horiz = mag
        self.output.east = x2-x1
        self.output.nrth = y2-y1
        self.output.elev = self.tgt_pose.pose.pose.position.z - \
                                    self.ref_pose.pose.pose.position.z

        test_msg = Float64()
        test_msg.data = mag
        self.pub_test.publish(test_msg)


    def spit(self):
        self.output.header.stamp = rospy.Time().now()
        self.output.header.frame_id = '/odom'
        self.output.reference_frame = self.ref_pose.header.frame_id
        self.output.target_frame = self.tgt_pose.header.frame_id
        self.pub.publish(self.output)

        if self.DEBUG: print("ErrorNode::publishing..."), pp(self.output)

if __name__ == '__main__':
    rospy.init_node('pose_error_node')
    app = ErrorNode()
    rospy.spin()