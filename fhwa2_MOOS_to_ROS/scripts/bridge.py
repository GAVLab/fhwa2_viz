#!/usr/bin/env python
import sys
from time import sleep
from pprint import pprint
import copy
from math import sqrt

import LL2UTM # Coordinate system-related

#ROS Imports
import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
from visualization_msgs.msg import Marker # had to add module to manifest
import tf

#MOOS Imports
from pymoos.MOOSCommClient import MOOSCommClient

###########################################################################

class ALOG2RVIZ(MOOSCommClient):
    """Takes moos messages from an alog file and displays the data in rViz"""
    def __init__(self):
        MOOSCommClient.__init__(self)
        self.SetOnConnectCallBack(self.onConnect)
        self.SetOnMailCallBack(self.onMail)

        self.odometry_variables = ['zLat','zLong','zLatStdDev','zLongStdDev']
        # self.fingerprint_variables = ['zGyroX_gXbow440','zGyroY_gXbow440','zGyroZ_gXbow440',
        #                               'zAccelX_gXbow440','zAccelY_gXbow440','zAccelZ_gXbow440']
        # self.misc_variables = ['zCourse','zHorizSpeed']
        self.desired_variables = self.odometry_variables # will expand later

        # Odom init
        self.odom_msgs = {}
        self.odom_msgs_count = {}
        self.odom_offset = {} # accounts for origin in UTM being located off yonder
        self.LatLong_holder = {} # must have both meas to convert to UTM
        self.new_LatLong = True
        self.first_odom = True
        self.odom_publisher = rospy.Publisher("/novatel/odom", Odometry)

        # Error ellipse, Vehicle model - init
        rospy.Subscriber("/novatel/odom", Odometry, self.pub_at_position)
        self.curpos_publisher = rospy.Publisher('/novatel/error_ellipse', Marker)


    def onConnect(self):
        print("In onConnect")
        for var in self.odometry_variables: # expand later
            self.Register(var) #defined in MOOSCommClient.py

    def onMail(self):
        print("In onMail")
        messages = self.FetchRecentMail()
        for message in messages:
            self.handle_msg(message)
        return True

    
    def handle_msg(self, msg):
        if msg.IsDouble():
            var_type = "Double"
            value = str(msg.GetDouble()) #store all values as strings until handled specifically
        elif msg.IsString():
            var_type = "String"
            value = msg.GetString()
        else:
            rospy.logwarn('wtf? Unknown variable type')

        time = msg.GetTime()
        name = msg.GetKey() # type of measurement "z______"
        if name in self.desired_variables: # where desired messages are scooped
            #send to appropriate variable handler
            if name in self.odometry_variables:
                self.handle_odom_var(name, var_type, value, time)
            # elif #..... other types of msgs to be done later
        else:
            rospy.logwarn("Unhandled msg: %(name)s of type %(var_type)s carries value %(value)s" %locals())

    
    def handle_odom_var(self, name, var_type, value, time):
        # Need to include covariance info from here throughout
        time = int(time*1000.0)/1000.0 #rounding to 3 decimal places so that the msg will groove...

        # remove this later when able to center to current position in rViz
        if self.first_odom: # deal with large coordinates by zeroing to first location measurment
            if name == "zLat":
                self.odom_offset['lat'] = float(value)
            elif name == "zLong":
                self.odom_offset['lon'] = float(value)
            if len(self.odom_offset) == 2:
                (E_offset, N_offset) = LL2UTM.convert(self.odom_offset['lat'],self.odom_offset['lon'])
                self.odom_offset = {}
                self.odom_offset['E'] = E_offset
                self.odom_offset['N'] = N_offset
                self.first_odom = False
        # this function should only receive msgs with name 'zLat' & 'zLong'
        # So missing info should be there
        if name not in self.LatLong_holder: 
            self.LatLong_holder[name] = dict([['var_type', var_type],
                                              ['value', float(value)],
                                              ['time', time]])#need to make sure time stamp is the same        
        # Problems might arise here if time steps and info flow doesn't match up
        if ('zLat' in self.LatLong_holder) and ('zLong' in self.LatLong_holder) and ('zLatStdDev' in self.LatLong_holder) and ('zLongStdDev' in self.LatLong_holder): # time step has all required infos
            time_lat = self.LatLong_holder['zLat']['time']
            time_lon = self.LatLong_holder['zLong']['time']
            if time_lat != time_lon:
                rospy.logwarn("Lat/Long mismatch:: time steps aren't being handled properly")

            # Position
            (Easting, Northing) = LL2UTM.convert(self.LatLong_holder['zLat']['value'], 
                                                 self.LatLong_holder['zLong']['value'])
            
            # Covariances
            LatStdDev_temp = self.LatLong_holder['zLat']['value'] + self.LatLong_holder['zLatStdDev']['value']
            LongStdDev_temp = self.LatLong_holder['zLong']['value'] - self.LatLong_holder['zLongStdDev']['value'] # Longitude will always be negative in UTM zone 16
            (EastingStdDev_temp, NorthingStdDev_temp) = LL2UTM.convert(LatStdDev_temp, LongStdDev_temp)
            EastingStdDev = abs(EastingStdDev_temp - Easting)
            NorthingStdDev = abs(NorthingStdDev_temp - Northing)

            # Consolidate into packaging dictionary
            self.NE_holder = {}
            self.NE_holder['N'] = Northing
            self.NE_holder['E'] = Easting
            self.NE_holder['Nsd'] = NorthingStdDev
            self.NE_holder['Esd'] = EastingStdDev
            self.NE_holder['time'] = self.LatLong_holder['zLong']['time']

            self.package_odom_var(self.NE_holder) # Send to shipping function
            self.LatLong_holder = {} # clear holder for location at next time step
            del self.NE_holder

    
    def package_odom_var(self, NE_holder):
        time = NE_holder['time']
        # Assume that the odom msg for this time step doesn't yet exist, create it
        self.odom_msgs[time] = Odometry()
        self.odom_msgs[time].header.stamp = rospy.Time(time)
        self.odom_msgs[time].header.frame_id = "odom"
        # when NE_holder reaches this function, it should have all the necessary info
        self.odom_msgs[time].pose.pose.position.x = NE_holder['E'] - self.odom_offset['E'] # return later and put at true position 
        self.odom_msgs[time].pose.pose.position.y = NE_holder['N'] - self.odom_offset['N'] # return later and put at tru position
        self.odom_msgs[time].pose.pose.position.z = 0 # constrain to xy axis for top-down view (this may not need to be stated)

        self.odom_msgs[time].pose.covariance[0] = NE_holder['Nsd']
        self.odom_msgs[time].pose.covariance[7] = NE_holder['Esd']
        self.odom_msgs[time].pose.covariance[14] = 0 # constrain to xy axis for top-down view (this may not need to be stated)

        # Send to positions display function
        self.odom_publisher.publish(self.odom_msgs[time]) # ship it! # this action moved to do_current position

        # send to error ellipse & vehicle model function
        self.pub_at_position(time)

        # tell camera tf where the look
        self.cameraFollow_tf(time)

        del self.odom_msgs[time]


    def pub_at_position(self, time):
        """ Handles necessary information for displaying error ellipses
        """
        marker = Marker()
        pub = self.curpos_publisher
        msg = self.odom_msgs[time]
        marker.header = msg.header
        marker.id = 0 # enumerate subsequent markers here
        marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
        marker.pose = msg.pose.pose
        marker.lifetime = rospy.Duration() # will last forever unless modified

        # Error Ellipse
        marker.ns = "Error_Ellipses"
        marker.type = Marker.CYLINDER     
        marker.scale.x = sqrt(msg.pose.covariance[0])
        marker.scale.y = sqrt(msg.pose.covariance[7])
        marker.scale.z = 0.000001
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6 # transparency
                
        pub.publish(marker)
        
        # Vehicle Model
        marker.ns = "vehicle_model"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.MODIFY
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.mesh_resource = "package://fhwa2_MOOS_to_ROS/mesh/2004_Infiniti_G35_sedan.dae"
        marker.mesh_use_embedded_materials = False

        pub.publish(marker)


    def cameraFollow_tf(self, time):
        msg = self.odom_msgs[time]
        br = tf.TransformBroadcaster()

        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), # send zjj in case
                         (0, 0, 0, 1), # this is a unit quaternion
                         msg.header.stamp,
                         "base_footprint", # child frame
                         "odom") #parent frame


#############################################################

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