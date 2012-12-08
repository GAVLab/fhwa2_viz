#!/usr/bin/env python
"""
This class puts a map of markers and meshes in rViz
It does not connect to MOOS

Author: Robert Cofield, for GAVLab, 7/19/2012
"""
import sys, os
from yaml import load
import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
from math import pi, sin, cos, tan, sqrt
from util import GPS
from pprint import pprint as pp


class MAP2RVIZ(object):
    def __init__(self):
        object.__init__(self)
        self.gps = GPS()
        self.get_config()
        self.set_publishers()
        self.create_map() # create marker arrays of the stripes and lane centers
        print('mapBridge: Map markers published - you should see the lane/stripe markers once running')
        self.create_map_mesh()
        print('mapBridge: Map mesh has been published - you should see it once running')

    def get_config(self):
        """startup function"""
        self.prefix = rospy.get_param('~prefix')
        # self.UTMdatum = {'E': float(rospy.get_param("~UTMdatum_E")),
        #                  'N': float(rospy.get_param("~UTMdatum_N"))}
        self.UTMdatum = rospy.get_param('/UTMdatum')
        self.survey_stripe_locs = rospy.get_param("~survey_stripe_locs").split(', ')
        self.survey_center_locs = rospy.get_param("~survey_center_locs").split(', ')
        self.track_mesh_resource = rospy.get_param("~track_mesh_resource")
        print('\ntrack_mesh_resource: %s' % self.track_mesh_resource)
        print('track_mesh_resource: %s' % str(bool(self.track_mesh_resource)))
        self.marking_mesh_resource = rospy.get_param("~marking_mesh_resource")

        self.sign_mesh_resource = rospy.get_param('~sign_mesh_resource')
        self.sign_file_loc = rospy.get_param('~survey_sign_locs')

    def set_publishers(self):
        """startup function"""
        self.map_stripe_publisher = rospy.Publisher('/map/survey_stripes', MarkerArray, latch=True)
        self.map_lane_publisher = rospy.Publisher('/map/survey_lanes', MarkerArray, latch=True)
        self.track_mesh_publisher = rospy.Publisher('/map/mesh', MarkerArray, latch=True)
        self.sign_publisher = rospy.Publisher('/map/signs', MarkerArray, latch=True)

    def survey(self):
        """
        reads "DecLat DecLon \n" text file and groups into 2 lists
        e.g., stripes = [[lat1, lon1], [lat2, lon2], ...]
        """
        stripes = []
        centers = []
        signs = []

        for stripe_file_loc in self.survey_stripe_locs:
            stripe_file = open(os.path.join(self.prefix, stripe_file_loc), 'rU')
            for line in stripe_file:
                line = line[0:-2] # remove the '\n' at the end
                pt_list = line.split(' ')
                stripes.append(pt_list)

        for center_file_loc in self.survey_center_locs:
            center_file = open(os.path.join(self.prefix, center_file_loc), 'rU')
            for line in center_file:
                line = line[0:-2] # remove the '\n' at the end
                pt_list = line.split(' ')
                centers.append(pt_list)

        sign_file = open(os.path.join(self.prefix, self.sign_file_loc), 'rU')
        for line in sign_file:
            line = line[0:-2]
            pt_list = line.split(' ')
            signs.append(pt_list)

        return stripes, centers, signs

    def create_map(self):
        """
        This function collects the survey data and puts it into marker arrays ready to be published in rviz
        """
        (stripes, centers, signs) = self.survey()
        
        self.map_stripe_array = MarkerArray()
        self.map_lane_array = MarkerArray()
        self.sign_array = MarkerArray()
        
        NCAT_id = 0
        
        ### Stripes ################################################################
        for pt in stripes:
            lat = float(pt[0])
            lon = float(pt[1])
            (east, nrth, _), info = self.gps.lla2utm((lat, lon, 0)) # convert to UTM
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.id = NCAT_id # enumerate subsequent markers here
            marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
            marker.lifetime = rospy.Duration() # will last forever unless modified
            marker.ns = "stripes"
            marker.type = Marker.CUBE
            marker.pose.position.x = east - float(self.UTMdatum['E']) 
            marker.pose.position.y = nrth - float(self.UTMdatum['N']) 
            marker.pose.position.z = 0 # zero is a novatel mount level
            marker.color.r = 255
            marker.color.g = 255
            marker.color.b = 0
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.4
            marker.mesh_use_embedded_materials = False
            self.map_stripe_array.markers.append(marker)
            
            NCAT_id += 1
        self.map_stripe_publisher.publish(self.map_stripe_array) # publish stripes as markers
        print('mapBridge: Stripe Markers have been printed') 
        del marker

        ### Lane Centers ###########################################################
        for pt in centers:
            lat = float(pt[0])
            lon = float(pt[1])
            (east, nrth, _), _ = self.gps.lla2utm((lat, lon, 0)) # convert to UTM
            
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.id = NCAT_id # enumerate subsequent markers here
            marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
            marker.lifetime = rospy.Duration() # will last forever unless modified
            marker.ns = "lane_centers"
            marker.type = Marker.SPHERE
            marker.pose.position.x = east - float(self.UTMdatum['E']) 
            marker.pose.position.y = nrth - float(self.UTMdatum['N']) 
            marker.pose.position.z = 0 # zero is a novatel mount level
            marker.color.r = 255
            marker.color.g = 255
            marker.color.b = 255
            marker.color.a = 0.75
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.mesh_use_embedded_materials = False
            self.map_lane_array.markers.append(marker)
            
            NCAT_id += 1
        self.map_lane_publisher.publish(self.map_lane_array) # publish lane centers as markers
        print('mapBridge: Lane Center Markers have been printed')

        ### Signs ##############################################################
        for pt in signs:
            lat = float(pt[0])
            lon = float(pt[1])
            (east, nrth, _), _ = self.gps.lla2utm((lat, lon, 0))

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.id = NCAT_id
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.ns = "signs"
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_use_embedded_materials = False
            marker.pose.position.x = east - float(self.UTMdatum['E']) 
            marker.pose.position.y = nrth - float(self.UTMdatum['N']) 
            marker.pose.position.z = 0 # zero is a novatel mount level
            marker.mesh_resource = '//'.join(['file:', self.sign_mesh_resource])
            marker.color.r = 1
            marker.color.g = 255
            marker.color.b = 1
            marker.color.a = 1
            marker.scale.x = .0254
            marker.scale.y = .0254
            marker.scale.z = .0254
            self.sign_array.markers.append(marker)

            NCAT_id += 1
        self.sign_publisher.publish(self.sign_array)


    def create_map_mesh(self):
        """Puts a blender mesh of the paement and stripes (continuous) into rviz"""
        marker_array = MarkerArray()

        ### Track Pavement Publisher ###########################################
        if self.track_mesh_resource:
            marker = Marker()
            marker.header.frame_id = 'odom' # publish in static frame
            marker.id = 0
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration() # immortal unless changed
            marker.ns = "track_mesh_pavement"
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_use_embedded_materials = False
            marker.mesh_resource = '//'.join(['file:', self.track_mesh_resource])
            marker.pose.position.x = 0# - float(self.UTMdatum['E']) 
            marker.pose.position.y = 0# - float(self.UTMdatum['N']) 
            marker.pose.position.z = 0
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.r = 0.3
            marker.color.g = 0.3
            marker.color.b = 0.3
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        ### Lane Marking Publisher ##############################################
        if self.marking_mesh_resource:
            marker = Marker()
            marker.header.frame_id = 'odom' # publish in static frame
            marker.id = 1
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration() # immortal unless changed
            marker.ns = "track_mesh_lane_markings"
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_use_embedded_materials = False
            marker.mesh_resource = self.marking_mesh_resource
            marker.pose.position.x = 0# - float(self.UTMdatum['E']) 
            marker.pose.position.y = 0# - float(self.UTMdatum['N']) 
            marker.pose.position.z = 0
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.r = 255
            marker.color.g = 255
            marker.color.b = 0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        self.track_mesh_publisher.publish(marker_array)


################################################################################
def main():
    rospy.init_node('moos2rviz_survey')
    app = MAP2RVIZ()
    rospy.spin()

if __name__ == '__main__':
    main()