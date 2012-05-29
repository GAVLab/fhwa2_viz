#!/usr/bin/env python
import sys
from time import sleep
from pprint import pprint
import copy
from math import sqrt, pi, radians, degrees

import LL2UTM # Coordinate system-related

#ROS Imports
import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
from visualization_msgs.msg import Marker # had to add module to manifest
import tf
from tf.transformations import quaternion_from_euler as qfe

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

        self.odometry_variables = ['zLat','zLong','zLatStdDev','zLongStdDev','zCourse']
        # self.fingerprint_variables = ['zGyroX_gXbow440','zGyroY_gXbow440','zGyroZ_gXbow440',
        #                               'zAccelX_gXbow440','zAccelY_gXbow440','zAccelZ_gXbow440']
        # self.misc_variables = [,'zHorizSpeed']
        self.desired_variables = self.odometry_variables # will expand later

        # Odom init
        self.odom_msgs = {}
        self.odom_msgs_count = {}
        self.LatLong_holder = {} # must have both meas to convert to UTM
        self.new_LatLong = True
        self.first_odom = True
        self.odom_publisher = rospy.Publisher("/novatel/odom", Odometry)

        # Error ellipse, Vehicle model - init
        rospy.Subscriber("/novatel/odom", Odometry, self.pub_at_position)
        self.curpos_publisher = rospy.Publisher('/novatel/error_ellipse', Marker)

        # Map track
        self.map_publisher = rospy.Publisher('/track_survey', Marker, latch=True)
        marker = Marker()
        marker.header.frame_id = '/odom'
        marker.id = 0 # enumerate subsequent markers here
        marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
        marker.lifetime = rospy.Duration() # will last forever unless modified
        marker.ns = "track_survey"
        marker.type = Marker.MESH_RESOURCE
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1
        # Debugging
        marker.scale.x = 100
        marker.scale.y = 100
        marker.scale.z = 100
        marker.mesh_resource = "package://fhwa2_MOOS_to_ROS/mesh/NCAT_UTM_Plane_10_linesOnly_stripesOnly.dae"
        marker.mesh_use_embedded_materials = False
        self.map_publisher.publish(marker)


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

        # this function should only receive msgs with name 'zLat' & 'zLong' & 'zCourse'
        # So missing info should be there
        if name not in self.LatLong_holder: 
            self.LatLong_holder[name] = dict([['var_type', var_type],
                                              ['value', float(value)],
                                              ['time', time]])#need to make sure time stamp is the same        
        # Problems might arise here if time steps and info flow doesn't match up
        all_present = True # Now determine if there's any msgs missing
        for var_name in self.odometry_variables:
            if var_name not in self.LatLong_holder:
                all_present = False
        if all_present: # time step has all required infos
            time_lat = self.LatLong_holder['zLat']['time']
            time_lon = self.LatLong_holder['zLong']['time']
            time_crs = self.LatLong_holder['zCourse']['time']
            if (time_lat != time_lon) or (time_lat != time_crs):
                rospy.logwarn("Lat/Long/Course mismatch:: time steps aren't being handled properly")
            
            # Position Conversions - !!! Currently most assuredly incorrect !!!
            (Easting, Northing) = LL2UTM.convert(self.LatLong_holder['zLat']['value'], 
                                                 self.LatLong_holder['zLong']['value'])
            
            # Covariances - !!! Currently most assuredly incorrect !!!
            LatStdDev_temp = self.LatLong_holder['zLat']['value'] + self.LatLong_holder['zLatStdDev']['value']
            LongStdDev_temp = self.LatLong_holder['zLong']['value'] - self.LatLong_holder['zLongStdDev']['value'] # Longitude will always be negative in UTM zone 16
            (EastingStdDev_temp, NorthingStdDev_temp) = LL2UTM.convert(LatStdDev_temp, LongStdDev_temp)
            EastingStdDev = abs(EastingStdDev_temp - Easting)
            NorthingStdDev = abs(NorthingStdDev_temp - Northing)
            
            print('EastingStdDev: %(EastingStdDev)f      EastingStdDev_temp: %(EastingStdDev_temp)f' %locals())
            print('NorthingStdDev %(NorthingStdDev)f      NorthingStdDev_temp: %(NorthingStdDev_temp)f' %locals())

            # Consolidate into packaging dictionary & send off
            self.NE_holder = {}
            self.NE_holder['N'] = Northing
            self.NE_holder['E'] = Easting
            self.NE_holder['Nsd'] = NorthingStdDev
            self.NE_holder['Esd'] = EastingStdDev
            self.NE_holder['crs'] = radians(self.LatLong_holder['zCourse']['value']) # Will orient odom arrows to velocity direction
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
        self.odom_msgs[time].pose.pose.position.x = NE_holder['E']
        self.odom_msgs[time].pose.pose.position.y = NE_holder['N']
        self.odom_msgs[time].pose.pose.position.z = 0 # constrain to xy axis for top-down view (this may not need to be stated)
        quat = qfe(-pi, -pi, -NE_holder['crs']-pi/2)
        self.odom_msgs[time].pose.pose.orientation.x = quat[0]
        self.odom_msgs[time].pose.pose.orientation.y = quat[1]
        self.odom_msgs[time].pose.pose.orientation.z = quat[2]
        self.odom_msgs[time].pose.pose.orientation.w = quat[3]
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
        marker.color.a = 0.5 # transparency            
        pub.publish(marker)
        
        # Vehicle Model
        marker.ns = "vehicle_model"
        marker.type = Marker.MESH_RESOURCE
        marker.scale.x = 0.0254 # artifact of sketchup export
        marker.scale.y = 0.0254 # artifact of sketchup export
        marker.scale.z = 0.0254 # artifact of sketchup export
        marker.color.r = .5
        marker.color.g = .5
        marker.color.b = .5
        marker.color.a = .7
        marker.mesh_resource = "package://fhwa2_MOOS_to_ROS/mesh/infiniti.dae"
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
