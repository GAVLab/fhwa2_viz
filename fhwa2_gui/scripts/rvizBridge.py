#!/usr/bin/env python
# Robert Cofield, GAVLab
# Python v2.7.3
# from pymoos.MOOSCommClient import MOOSCommClient # used this when pymoos wouldn't build on hp under ros
from pprint import pprint as pp
import sys, os
from time import sleep, time
from yaml import load
from math import radians, sqrt, pi, degrees
# from mapping import ll2utm

#ROS Imports
import roslib
roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from nav_msgs.msg import Odometry # this will need to be repeated for other message types?
from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
import tf
from tf.transformations import quaternion_from_euler as qfe
from util import GPS

#MOOS Imports
# sys.path.append('../../../MOOS-ros-pkg/MOOS/pymoos/python') # location of one file named MOOSCommClient.py (other located in bin)

# sys.path.append('../../../MOOS-ros-pkg/MOOS/pymoos/lib')
from pymoos.MOOSCommClient import MOOSCommClient
from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg


class MOOS2RVIZ:
    """Takes moos messages from an onboard moos db and displays the data in rViz"""
    def __init__(self, config):
        self.get_config(config)
        self.set_publishers()
        self.navpy_gps = GPS()

        self.holder = {}
        self.make_holder_nones()
        self.time_pub_tol = 1.0

    def make_holder_nones(self):
        for var in self.desired_variables:
            self.holder[var] = [None] * 3 # [time(float), var_type(string), value(float)] for now only double/float values supported

    ### Init-related Functions #################################################
    def get_config(self, config):
        """saves the yaml config file info as instance attributes, utilizes
        values from the ROS parameter server
        """
        self.moosapp_name = config['moosapp_name']
        self.sensor_name = config["sensor_name"]
        self.freq_max = config["freq_max"]
        self.UTMdatum = config["UTMdatum"]
        self.coord_sys = config["coord_sys"]
        self.desired_variables = config["desired_variables"]
        self.color = config["color"]
        self.legend_text_height = config["legend_text_height"]
        self.legend_text = config["display_name"]
        self.ismaster = config["dictate_pos"] # this instance will govern the current position
        if self.ismaster:
            self.veh_mesh_resource = config["veh_mesh_resource"]
        else:
            self.veh_mesh_resource = None
        print('rvizBridde '+self.moosapp_name+': desired variables are')
        pp(self.desired_variables)


    def set_publishers(self):
        rospy.Subscriber("/moos/"+self.moosapp_name, MOOSrosmsg, self.handle_msg)
        odom_topic = '/'.join([self.moosapp_name, 'odom'])
        self.odom_publisher = rospy.Publisher(odom_topic, Odometry)
        ell_topic = '/'.join([self.moosapp_name, 'error_ellipse'])
        self.ell_publisher = rospy.Publisher(ell_topic, Marker)
        legend_topic = '/'.join([self.moosapp_name, 'legend'])
        self.legend_publisher = rospy.Publisher(legend_topic, Marker)
        if self.ismaster: # this instance dictates accepted position (veh disp mesh)
            curpos_topic = '/'.join([self.moosapp_name, 'current_position'])
            self.curpos_publisher = rospy.Publisher(curpos_topic, Marker) # even though this is at the same position as the novatel error ellipse, we want it to have a different name in case the integrated solution is different
            rospy.Subscriber(odom_topic, Odometry, self.pub_at_position) # put the vehicle model at the accepted position solution

        combined_topic = '/'.join([self.moosapp_name, "the_3"])
        self.combined_publisher = rospy.Publisher(combined_topic, MarkerArray)
        print('rvizBridde '+self.moosapp_name+': Publishers and Subscribers set')
    ############################################################################

    def cameraFollow_tf(self, odom_msg):
        msg = odom_msg
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),\
                         (0, 0, 0, 1), # this is a unit quaternion\
                         msg.header.stamp,"base_footprint", "odom") 
                            #time,    #child frame , #parent frame

    ##### Mailroom Functions ###################################################
    def handle_msg(self, msg):
        """
        takes a MOOSrosmsg and passes usable info to gather_odom_var
        """
        # print("rvizBridge: " + self.moosapp_name + ": msg received "+msg.MOOStype)
        if (msg.MOOSsource == self.sensor_name) and (msg.MOOSname in self.desired_variables):
            var_type = msg.MOOStype
            if var_type == "Double":
                value = str(msg.MOOSdouble) #store all values as strings until handled specifically
            elif var_type == "String":
                rospy.logwarn("Strings not yet supported")
            else:
                rospy.logwarn('wtf? Unknown variable type')
            # obtain info from msg via functions defined in pyMOOSMsg.cpp
            time = msg.MOOStime # this may be grabbing the wrong time (not from the msg)
            name = msg.MOOSname # type of measurement "z______" string
            sens = msg.MOOSsource # will yield "g______" string

            self.gather_odom_var(name, var_type, value, time)
            print("\n\nrvizBridge: " + self.moosapp_name + ': handle_msg sent to gather_odom_var:')
            print('\tname: '+name)
            print('\ttime: '+str(time))
            print('\tsens: '+sens)
            print('\tvalue: '+str(value))
        # else:
            # print('rvizBridge: '+self.moosapp_name+': handle_msg rejected msg ')
            # pp(msg)
        # else:
        #     rospy.logwarn("handle_msg :: Unhandled msg: %(name)s of type %(var_type)s carries value %(value)s" % locals())


    def gather_odom_var(self, name, var_type, value, time):
        """
        This function will only invoke publishing when it can send off all
        necessary info for a single source at once
        """
        print('rvizBridge: '+self.moosapp_name+": In gather_odom_var for "+name)
        time = int(time*1000.0)/1000.0 #rounding to 3 decimal places 
        print('rvizBridge: '+self.moosapp_name+": In gather_odom_var the preexisting state of self.holder is --")
        pp(self.holder)

        self.holder[name] = [time, var_type, value]
        shippable = [False]*len(self.desired_variables)
        for des_var_ind in range(len(self.desired_variables)):
            if all(self.holder[self.desired_variables[des_var_ind]]):
                shippable[des_var_ind] = True
        if all(shippable):
            print('\nrvizBridge: '+self.moosapp_name+": gather_odom_var IS SHIPPING NOW\n")
            skateboard = []
            for var in self.desired_variables:
                skateboard.append( self.holder[var][2] )
            self.convert_odom_var(skateboard)
            self.make_holder_nones()


    def convert_odom_var(self, skateboard):
        """
        Once all the information for a single odom msg from a single sensor
        (source) is built together, this function converts it to UTM in a 
        special UTM holder and sends it to the publishing function: mailroom.package_odom_var()
        Covariances - Lat/Lon Std Dev are output in meters already
        converts course to radians for ROS; Will orient odom arrows to velocity direction

        FIX ECEF STD DEVS!!!!!!!!!!!!!!!!!!!!!!!!!

        """
        ## FIXME here an assumption is made about the order of variables - robustify later
        print('rvizBridge: '+self.moosapp_name+': convert_odom_var: SKATEBOARD RECIEVED --')
        pp(skateboard)
        if self.coord_sys == 'ECEF':
            (E, N, _), _ = self.navpy_gps.ecef2utm((float(skateboard[0]),
                                                    float(skateboard[1]),
                                                    float(skateboard[2])))
            
            Xstray = float(skateboard[0])+float(skateboard[3])
            Ystray = float(skateboard[1])-float(skateboard[4]) # ASSUME Y ECEF IS NEGATIVE
            Zstray = float(skateboard[2])+float(skateboard[5])
            print ('Xstray, Ystray, Zstray')
            pp(Xstray), pp(Ystray), pp(Zstray)
            (Estray, Nstray, _), _ = self.navpy_gps.ecef2utm((Xstray, Ystray, Zstray))
            Esd = Estray - E
            Nsd = Nstray - N

            UTMtoPub = {}
            UTMtoPub['N'] = N - self.UTMdatum['N']
            UTMtoPub['E'] = E - self.UTMdatum['E']
            UTMtoPub['Nsd'] = float(Nsd)
            UTMtoPub['Esd'] = float(Esd)
            UTMtoPub['crs'] = radians(float(skateboard[-1]))

            print('rvizBridge: '+self.moosapp_name+': convert_odom_var: UTMtoPub --')
            pp(UTMtoPub)
            self.package_odom_var(UTMtoPub)
        else:
            raise Exception('Only ECEF implemented thus far')
        # print("{} - {} = {}".format(Northing, self.UTMdatum['N'], Northing - self.UTMdatum['N']))


    def package_odom_var(self, UTMtoPub):
        """
        Puts odom var (from UTM N & E) into a ros message and sends to the publishing decider.
            -does all the ROS packaging common to all sources
            -each individual legend is now
        """
        print('rvizBridge: '+self.moosapp_name+': In package_odom_var')
        combined_array = MarkerArray()

        ### Odometry Arrows ####################################################
        odom_msg = Odometry()
        # odom_msg.header.stamp = rospy.Time(UTMtoPub['time'])
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom" # may need to expand?
        odom_msg.pose.pose.position.x = UTMtoPub['E']
        odom_msg.pose.pose.position.y = UTMtoPub['N']
        odom_msg.pose.pose.position.z = 1.55
        # make a quaternion :: came from trial-and-error...
        quat = qfe(-pi, -pi, -UTMtoPub['crs']-pi/2)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.pose.covariance[0] = UTMtoPub['Nsd']
        odom_msg.pose.covariance[7] = UTMtoPub['Esd']
        odom_msg.pose.covariance[14] = 0
        self.odom_publisher.publish(odom_msg)

        combined_array.markers.append(odom_msg)

        ### Error Ellipses #####################################################
        ## pulls from the odom msg --
        ell_marker = Marker()
        ell_marker.header = odom_msg.header # clarify?
        ell_marker.id = 0 # enumerate subsequent markers here 
        ell_marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
        ell_marker.pose = odom_msg.pose.pose # put at same place as its odom arrow
        ell_marker.lifetime = rospy.Duration() # will last forever unless modified
        ell_marker.ns = ''.join(["Error_Ellipses", '__', self.sensor_name, '__', self.moosapp_name])
        ell_marker.type = Marker.CYLINDER     
        ell_marker.scale.x = sqrt(odom_msg.pose.covariance[0]) # not visible unless scaled up
        ell_marker.scale.y = sqrt(odom_msg.pose.covariance[7]) # not visible unless scaled up
        ell_marker.scale.z = 0.000001 # We just want a disk
        ell_marker.color.r = self.color['r']
        ell_marker.color.g = self.color['g']
        ell_marker.color.b = self.color['b']
        ell_marker.color.a = 0.95
        self.ell_publisher.publish(ell_marker)

        combined_array.markers.append(ell_marker)

        ### Legend #############################################################
        legend_marker = Marker()
        legend_marker.header = odom_msg.header
        legend_marker.id = 0
        legend_marker.ns = ''.join(["Error_Ellipses", '__', self.sensor_name, '__', self.moosapp_name])
        legend_marker.type = Marker.TEXT_VIEW_FACING
        legend_marker.text = self.legend_text
        legend_marker.action = Marker.MODIFY
        legend_marker.pose = odom_msg.pose.pose
        legend_marker.pose.position.z = self.legend_text_height # elevate to spread
        legend_marker.scale.x = 0.5
        legend_marker.scale.y = 0.5
        legend_marker.scale.z = 0.6
        legend_marker.color.r = self.color['r']
        legend_marker.color.g = self.color['g']
        legend_marker.color.b = self.color['b']
        legend_marker.color.a = 1.0
        self.legend_publisher.publish(legend_marker)

        combined_array.markers.append(legend_marker)
        # self.combined_publisher.publish(combined_array)

        # tell camera tf where the look if master ##############################
        if self.ismaster: # update the vehicle mesh position
            self.pub_at_position(odom_msg)
            self.cameraFollow_tf(odom_msg)
            # self.FPV_tf(odom_msg) # add a view

    def pub_at_position(self, odom_msg):
        """
        Handles necessary information for displaying things at 
                                ACCEPTED (NOVATEL) POSITION SOLUTION:
            -vehicle mesh 
        !!!this should only be invoked when the accepted (novatel) position 
        is updated
        """
        ### G35 Mesh #############################################################
        marker = Marker()
        pub = self.curpos_publisher
        marker.header = odom_msg.header
        marker.id = 0 # enumerate subsequent markers here
        marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
        marker.pose = odom_msg.pose.pose
        marker.pose.position.z = 1.55
        marker.lifetime = rospy.Duration() # will last forever unless modified
        marker.ns = ''.join(["vehicle_model", '__', self.sensor_name, '__', self.moosapp_name])
        marker.type = Marker.MESH_RESOURCE
        marker.scale.x = 0.0254 # artifact of sketchup export
        marker.scale.y = 0.0254 # artifact of sketchup export
        marker.scale.z = 0.0254 # artifact of sketchup export
        marker.color.r = .05
        marker.color.g = .05
        marker.color.b = .05
        marker.color.a = .2
        marker.mesh_resource = self.veh_mesh_resource
        marker.mesh_use_embedded_materials = False
        pub.publish(marker)

################################################################################
################################################################################

def main():    
    ## setup config file
    config_file = sys.argv[1]
    if config_file[-4:] != 'yaml':
        print("Config file must be YAML format")
    stream = file(config_file,'r')
    this_config = load(stream) # loads as a dictionary

    #setup ROS node
    node_name = 'rvizBridge_'.join([this_config['moosapp_name']])
    rospy.init_node(node_name)

    #Setup App
    app = MOOS2RVIZ(this_config)
    rospy.spin()
    
if __name__ == '__main__':
    main()