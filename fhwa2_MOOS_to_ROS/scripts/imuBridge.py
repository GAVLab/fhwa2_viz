#!/usr/bin/env python
import roslib
roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from sensor_msgs.msg import Imu
from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg


class ImuBridge(object):
    """send XBow 440 to ROS"""
    def __init__(self):
        super(ImuBridge, self).__init__()
        self.sub = rospy.Subscriber('/moos/gMOOS2ROS', MOOSrosmsg, self.filter)
        self.pub = rospy.Publisher('/XBow440', Imu)

    def filter(self, msg):
        # print('In filter')
        if msg.MOOSname == 'zXBOW_gXbow440':
            self.handle_msg(msg)

    def handle_msg(self, msg_in):
        # print 'In handle_msg'
        msg_out = Imu()
        msg_out.header.stamp = rospy.Time(msg_in.MOOStime)
        # msg_out.header.frame_id = 'XBow440_link'
        msg_out.header.frame_id = 'rtk_base_link'
        s_vals = msg_in.MOOSstring.split('{')[1].split('}')[0].split(',')
        msg_out.linear_acceleration.x = float(s_vals[0])
        msg_out.linear_acceleration.y = float(s_vals[1])
        msg_out.linear_acceleration.z = float(s_vals[2])
        msg_out.angular_velocity.x = float(s_vals[3])
        msg_out.angular_velocity.y = float(s_vals[4])
        msg_out.angular_velocity.z = float(s_vals[5])

        self.pub.publish(msg_out)


if __name__ == '__main__':
    rospy.init_node('xBow_bridge')
    app = ImuBridge()
    rospy.spin()