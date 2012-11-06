#!/usr/bin/env python
import sys, os
import roslib
roslib.load_manifest('fhwa2_gui')
import rospy
from fhwa2_gui.msg import PoseError
from nav_msgs.msg import Odometry


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
        self.ref_sub = rospy.Subscriber(self.ref_topic, Odometry, self.onRefUpdate)
        self.tgt_sub = rospy.Subscriber(self.tgt_topic, Odometry, self.onTgtUpdate)
        # Data holders
        self.ref_pose = Odometry()
        self.tgt_pose = Odometry()
        self.error_out = PoseError()


    def onRefUpdate(self, msg):
        if self.DEBUG: print('In ErrorNode::onRefUpdate')
        self.ref_pose = msg


    def onTgtUpdate(self, msg):
        if self.DEBUG: print('In ErrorNode::onRefUpdate')
        self.tgt_pose = msg
        self.crunch()
        self.spit()


    def crunch(self):
         

    def spit(self):
        pass

        

if __name__ == '__main__':
    rospy.init_node('pose_error_node')
    app = ErrorNode()
    rospy.spin()