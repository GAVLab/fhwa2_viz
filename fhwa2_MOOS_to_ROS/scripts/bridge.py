#!/usr/bin/env python
import sys
from time import sleep
from pprint import pprint
import copy

# from survey import survey

# Bridge module imports
import randmcnally
import mailroom

#ROS Imports
import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
import tf

#MOOS Imports
from pymoos.MOOSCommClient import MOOSCommClient
course = None

###########################################################################

class ALOG2RVIZ(MOOSCommClient):
    """Takes moos messages from an alog file and displays the data in rViz"""
    def __init__(self):
        MOOSCommClient.__init__(self)
        self.SetOnConnectCallBack(self.onConnect)
        self.SetOnMailCallBack(self.onMail)

        # Map track
        self.map_stripe_publisher = rospy.Publisher('/map/survey_stripes', MarkerArray, latch=True)
        self.map_lane_publisher = rospy.Publisher('/map/survey_lanes', MarkerArray, latch=True)
        # self.create_NCAT_map()
        randmcnally.create_map(self)
        self.map_stripe_publisher.publish(self.map_stripe_array)
        self.map_lane_publisher.publish(self.map_lane_array)
        print('Map has been published')

        # Odom init
        self.odometry_variables = ['zLat','zLong','zLatStdDev','zLongStdDev','zCourse']
        # self.fingerprint_variables = ['zGyroX_gXbow440','zGyroY_gXbow440','zGyroZ_gXbow440',
        #                               'zAccelX_gXbow440','zAccelY_gXbow440','zAccelZ_gXbow440']
        # self.misc_variables = [,'zHorizSpeed']
        self.desired_variables = self.odometry_variables # will expand later
        self.odom_msgs = {}
        self.odom_msgs_count = {}
        self.LatLong_holder = {} # must have both meas to convert to UTM
        self.new_LatLong = True
        self.first_odom = True
        self.odom_publisher = rospy.Publisher("/novatel/odom", Odometry)

        # Error ellipse, Vehicle model - init
        rospy.Subscriber("/novatel/odom", Odometry, mailroom.pub_at_position)
        self.curpos_publisher = rospy.Publisher('/novatel/current_position', Marker)


    def onConnect(self):
        print("In onConnect")
        for var in self.odometry_variables: # expand later
            self.Register(var) #defined in MOOSCommClient.py


    def onMail(self):
        print("In onMail")
        messages = self.FetchRecentMail()
        for message in messages:
            mailroom.handle_msg(self, message)
        return True


def main():
    #setup ROS node
    rospy.init_node('alog2rviz')

    #Setup MOOS App
    app = ALOG2RVIZ()
    app.Run("127.0.0.1", 9000, "alog2rviz")
    for x in range(10): # allow 1 second to connect to MOOSDB
        sleep(0.1)
        if app.IsConnected():
            print("Connected to MOOSDB")
            break
    # Make sure we Connected
    if not app.IsConnected():
        rospy.logerr("Failed to Connect to MOOSDB")
        sys.exit(-1)
    
    #Setup ROS Stuff
    # setup new types of ros topics
   
    #spin
    rospy.spin()
    
if __name__ == '__main__':
    main()
