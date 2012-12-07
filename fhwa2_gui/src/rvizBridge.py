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
# from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg
from fhwa2_gui.msg import ECEFpos


class MOOS2RVIZ:
    """Takes moos messages from an onboard moos db and displays the data in rViz"""
    def __init__(self):
        self.get_config()
        self.set_publishers()
        self.navpy_gps = GPS()

        self.holder = {}

    ### Init-related Functions #################################################
    def get_config(self):
        """saves the yaml config file info as instance attributes, utilizes
        values from the ROS parameter server
        """
        self.DEBUG = bool(rospy.get_param('~DEBUG'))
        self.tag = rospy.get_param('~tag')

        self.UTMdatum = {'E': float(rospy.get_param("~UTMdatum_E")),
                         'N': float(rospy.get_param("~UTMdatum_N"))}
        self.coord_sys = rospy.get_param("~coord_sys")
        self.color = {'r': int(rospy.get_param("/"+self.tag+"_color").split(',')[0])*255,
                      'g': int(rospy.get_param("/"+self.tag+"_color").split(',')[1])*255,
                      'b': int(rospy.get_param("/"+self.tag+"_color").split(',')[2])*255}
        self.legend_text_height = rospy.get_param("~legend_text_height")
        self.legend_text = rospy.get_param("~display_name")      
        self.veh_mesh_resource = rospy.get_param("~veh_mesh_resource")
        # self.err_ell_opacity = rospy.get_param('~err_ell_opacity')


    def set_publishers(self):
        rospy.Subscriber("/moos/solutions/"+self.tag, ECEFpos, self.handle_msg)
        
        odom_topic = '/'.join(['moos', self.tag, 'odom'])
        ell_topic = '/'.join(['moos',self.tag, 'error_ellipse'])
        legend_topic = '/'.join(['moos', self.tag, 'legend'])

        self.odom_publisher = rospy.Publisher(odom_topic, Odometry)
        self.ell_publisher = rospy.Publisher(ell_topic, Marker)
        self.legend_publisher = rospy.Publisher(legend_topic, Marker)
        
        curpos_topic = '/'.join(['moos', self.tag, 'current_position'])
        self.curpos_publisher = rospy.Publisher(curpos_topic, Marker) # even though this is at the same position as the novatel error ellipse, we want it to have a different name in case the integrated solution is different
        rospy.Subscriber(odom_topic, Odometry, self.pub_at_position) # put the vehicle model at the accepted position solution

        # combined_topic = '/'.join(['moos',self.tag, "the_3"])
        # self.combined_publisher = rospy.Publisher(combined_topic, MarkerArray)
        if self.DEBUG:
            print('rvizBridge '+self.tag+': Publishers and Subscribers set')
    ############################################################################

    def cameraFollow_tf(self, odom_msg):
        """
        reexamine if using urdf (base_footprint)
        """
        msg = odom_msg
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z),
                         (0, 0, 0, 1), # this is a unit quaternion\
                         msg.header.stamp,
                         ''.join([self.tag,"_base_link"]), "odom") 

    def handle_msg(self, msg):
        """
        message callback
        """
        skateboard = [msg.x, msg.y, msg.z, \
                      msg.x_covar, msg.y_covar, msg.z_covar, \
                      msg.orient, msg.header.stamp]
        self.convert_odom_var(skateboard)

    def convert_odom_var(self, skateboard):
        """
        Once all the information for a single odom msg from a single sensor
        (source) is built together, this function converts it to UTM in a 
        special UTM holder and sends it to the publishing function: mailroom.package_odom_var()
        Covariances - Lat/Lon Std Dev are output in meters already
        converts course to radians for ROS; Will orient odom arrows to velocity direction

        skateboard = [X, Y, Z, XStdDev, YStdDev, ZStdDev, Course, MOOStime]
        
        # ASSUME Y ECEF IS NEGATIVE
        """
        ## FIXME here an assumption is made about the order of variables - robustify later
        if self.DEBUG:
            print('rvizBridge: '+self.tag+': convert_odom_var: SKATEBOARD RECIEVED --')
            pp(skateboard)
        if self.coord_sys == 'ECEF':
            (E, N, _), _ = self.navpy_gps.ecef2utm((float(skateboard[0]),
                                                    float(skateboard[1]),
                                                    float(skateboard[2])))
            Xstray = float(skateboard[0])+float(skateboard[3])
            Ystray = float(skateboard[1])-float(skateboard[4]) # ASSUME Y ECEF IS NEGATIVE
            Zstray = float(skateboard[2])+float(skateboard[5])
            (Estray, Nstray, _), _ = self.navpy_gps.ecef2utm((Xstray, Ystray, Zstray))
            Esd = Estray - E
            Nsd = Nstray - N

            UTMtoPub = {}
            UTMtoPub['N'] = N - self.UTMdatum['N']
            UTMtoPub['E'] = E - self.UTMdatum['E']
            UTMtoPub['Nsd'] = abs(float(Nsd))
            UTMtoPub['Esd'] = abs(float(Esd))
            UTMtoPub['crs'] = radians(float(skateboard[6]))
            UTMtoPub['time'] = skateboard[7]
            self.package_odom_var(UTMtoPub)
            if self.DEBUG:
                print('rvizBridge: '+self.tag+': convert_odom_var: UTMtoPub --')
                pp(UTMtoPub)
        else:
            raise Exception('Only ECEF implemented thus far')
        # print("{} - {} = {}".format(Northing, self.UTMdatum['N'], Northing - self.UTMdatum['N']))


    def package_odom_var(self, UTMtoPub):
        """
        Puts odom var (from UTM N & E) into a ros message and sends to the publishing decider.
            -does all the ROS packaging common to all sources
            -each individual legend is now
        """
        if self.DEBUG:
            print('rvizBridge: '+self.tag+': In package_odom_var')
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

        ### Error Ellipses #####################################################
        ell_marker = Marker()
        ell_marker.header = odom_msg.header # clarify?
        ell_marker.id = 0 # enumerate subsequent markers here 
        ell_marker.action = Marker.MODIFY # can be ADD, REMOVE, or MODIFY
        ell_marker.pose = odom_msg.pose.pose # put at same place as its odom arrow
        ell_marker.lifetime = rospy.Duration() # will last forever unless modified
        ell_marker.ns = "Error_Ellipses__"+ self.tag
        ell_marker.type = Marker.CYLINDER
        ell_marker.scale.x = abs(odom_msg.pose.covariance[0]) # not visible unless scaled up
        ell_marker.scale.y = abs(odom_msg.pose.covariance[7]) # not visible unless scaled up
        ell_marker.scale.z = 0.000001 # We just want a disk
        ell_marker.color.r = self.color['r']
        ell_marker.color.g = self.color['g']
        ell_marker.color.b = self.color['b']
        # ell_marker.color.a = self.err_ell_opacity
        ell_marker.color.a = 0.5
        self.ell_publisher.publish(ell_marker)

        ### Legend #############################################################
        legend_marker = Marker()
        legend_marker.header = odom_msg.header
        legend_marker.id = 0
        legend_marker.ns = ''.join(["Error_Ellipses", '__', self.tag, '__', self.tag])
        legend_marker.type = Marker.TEXT_VIEW_FACING
        legend_marker.text = self.legend_text
        legend_marker.action = Marker.MODIFY
        legend_marker.pose = odom_msg.pose.pose
        legend_marker.pose.position.z = self.legend_text_height # elevate to spread
        legend_marker.scale.x = .7
        legend_marker.scale.y = .7
        legend_marker.scale.z = .7
        legend_marker.color.r = self.color['r']/255
        legend_marker.color.g = self.color['g']/255
        legend_marker.color.b = self.color['b']/255
        legend_marker.color.a = .9
        self.legend_publisher.publish(legend_marker)

        self.cameraFollow_tf(odom_msg)
        self.pub_at_position(odom_msg)

    def pub_at_position(self, odom_msg):
        """
        Handles necessary information for displaying things at 
                                ACCEPTED (NOVATEL) POSITION SOLUTION:
            -vehicle mesh 
        !!!this should only be invoked when the accepted (novatel) position 
        is updated
        """
        ### G35 Mesh ###########################################################
        marker = Marker()
        marker.header = odom_msg.header
        marker.id = 0 # enumerate subsequent markers here
        marker.action = Marker.MODIFY
        marker.pose = odom_msg.pose.pose
        marker.pose.position.z = 1.55
        marker.lifetime = rospy.Duration() # will last forever unless modified
        marker.ns = ''.join(["vehicle_model", '__', self.tag, '__', self.tag])
        marker.type = Marker.MESH_RESOURCE
        marker.scale.x = 0.0254 # artifact of sketchup export
        marker.scale.y = 0.0254 # artifact of sketchup export
        marker.scale.z = 0.0254 # artifact of sketchup export
        marker.color.r = self.color['r']
        marker.color.g = self.color['g']
        marker.color.b = self.color['b']
        marker.color.a = .1
        marker.mesh_resource = '//'.join(['file:',self.veh_mesh_resource])
        marker.mesh_use_embedded_materials = False
        self.curpos_publisher.publish(marker)

################################################################################
################################################################################

def main():    
    rospy.init_node('fhwa2_rviz_bridge')
    app = MOOS2RVIZ()
    rospy.spin()

    
if __name__ == '__main__':
    main()