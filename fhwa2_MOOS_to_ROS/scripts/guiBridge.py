#!/usr/bin/env python
# Robert Cofield, GAVLab
# Python v2.7.3


import sys, os
from time import sleep
from pprint import pprint
import copy

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
# sys.path.append('../../../MOOS-ros-pkg/MOOS/pymoos/python') # location of one file named MOOSCommClient.py (other located in bin)
from pymoos.MOOSCommClient import MOOSCommClient
# sys.path.append('../../../MOOS-ros-pkg/MOOS/pymoos/lib')
# import MOOSCommClient


###########################################################################

class MOOS2RVIZ(MOOSCommClient):
    """Takes moos messages from an onboard moos db and displays the data in rViz"""
    def __init__(self):
        MOOSCommClient.__init__(self)
        self.SetOnConnectCallBack(self.onConnect)
        self.SetOnMailCallBack(self.onMail)

        ### Publishers
        self.map_stripe_publisher = rospy.Publisher('/map/survey_stripes', MarkerArray, latch=True)
        self.map_lane_publisher = rospy.Publisher('/map/survey_lanes', MarkerArray, latch=True)
        self.odom_novatel_publisher = rospy.Publisher("/novatel/odom", Odometry) # this is the accepted (combined) position solution
        self.odom_pennst_publisher = rospy.Publisher("/pennst/odom", Odometry) # component position solution
        self.odom_sri_publisher = rospy.Publisher("/sri/odom", Odometry) # component position solution
        self.odom_dsrc_publisher = rospy.Publisher("/dsrc/odom", Odometry) # component position solution
        # Error Ellipse Init
        self.ell_novatel_publisher = rospy.Publisher("/novatel/error_ellipse", Marker)
        self.ell_pennst_publisher = rospy.Publisher("/pennst/error_ellipse", Marker)
        self.ell_sri_publisher = rospy.Publisher("/sri/error_ellipse", Marker)
        self.ell_dsrc_publisher = rospy.Publisher("/dsrc/error_ellipse", Marker)
        # legend init
        self.legend_novatel_publisher = rospy.Publisher('novatel/legend', Marker)
        self.legend_pennst_publisher = rospy.Publisher('pennst/legend', Marker)
        self.legend_sri_publisher = rospy.Publisher('sri/legend', Marker)
        self.legend_dsrc_publisher = rospy.Publisher('dsrc/legend', Marker)
        # mesh of track from survey points
        self.track_mesh_publisher = rospy.Publisher('/map/mesh', Marker, latch=True)

        self.UTMdatum = dict([['E', 659300], ['N', 3607850]]) # roughly center of the track
        
        randmcnally.create_map(self) # create marker arrays of the stripes and lane centers
        self.map_stripe_publisher.publish(self.map_stripe_array) # publish stripes as markers
        self.map_lane_publisher.publish(self.map_lane_array) # publish lane centers as markers
        print('Map has been published - you should see the lane/stripe markers once running')

        # Publish track mesh
        randmcnally.create_map_mesh(self)
        # self.track_mesh_publisher.publish(self.map_mesh_marker)
        print('Map mesh has been published - you should see it once running')

        # Odom init
        self.desired_variables = ('zLat','zLong','zLatStdDev','zLongStdDev','zCourse') # we want these measurments from each sensor, below
        self.sensors = ('gNovatel', 'gPennSt', 'gSRI', 'gDSRC') # the 4 sources which will need to be displayed simultaneously
        # self.odom_msgs = {}
        # self.odom_msgs_count = {}
        # self.LatLong_holder = {} # must have both meas to convert to UTM; may need to initialize sub dictionaries?
        # for sens_str in self.sensors:
        #     self.LatLong_holder[sens_str] = {}

                # !! Multiple source message holding - debug only - revise later !! #
        self.gNovatel_holder = {}
        self.gPennSt_holder = {}
        self.gSRI_holder = {}
        self.gDSRC_holder = {}

        # odom & error ellipse colors - only sets the err ell colors, but these are in the config for the odom msgs
        self.rgb_novatel =  dict([['r', 0],     ['g', 150],   ['b',0]])         #black
        self.rgb_pennst =   dict([['r', 0],     ['g', 0],   ['b',127]])         #blue
        self.rgb_sri    =   dict([['r', 170],   ['g', 0],   ['b',127]])         #purple
        self.rgb_dsrc   =   dict([['r', 170],   ['g', 0],   ['b',0]])           #red

        # Vehicle model - init
        rospy.Subscriber("/novatel/odom", Odometry, mailroom.pub_at_position) # put the vehicle model at the accepted position solution
        self.curpos_publisher = rospy.Publisher('/novatel/current_position', Marker) # even though this is at the same position as the novatel error ellipse, we want it to have a different name in case the integrated solution is different


    # These functions required in every MOOS App

    def onConnect(self): 
        print("In onConnect")
        for var in self.desired_variables: # expand later
            self.Register(var) #defined in MOOSCommClient.py

    def onMail(self):
        print("In onMail")
        messages = self.FetchRecentMail()
        for message in messages:
            mailroom.handle_msg(self, message)
        return True


def main():
    #setup ROS node
    rospy.init_node('moos2rviz')

    #Setup MOOS App
    app = MOOS2RVIZ()
    app.Run("127.0.0.1", 9000, "moos2rviz") # change this to G comp (where MOOSDB is) currently BlackOak
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
    # curr = os.getcwd()
    main()
    # os.chdir(curr)