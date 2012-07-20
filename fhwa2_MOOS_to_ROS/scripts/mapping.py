#!/usr/bin/env python
"""
This file contain modules related to coordinate systems, and specifically the NCAT track map.
This is invoked a single time by an instance of mapBridge.py

Created 5/20/2012
Author: Robert Cofield
"""

def ll2utm(Lat, Long):
    """
    Converts decimal degree latitude & longitude to UTM Northing & Easting

    Adaptation of code from http://mahi.ucsd.edu/class233/data/Basic/LatLongUTMconversion.py
    Author: Robert Cofield
    20 May, 2012
    Only works for UTM Zone 16 (any letter designation), Northern Hemisphere
    must stay in same letter designation.
    """

    from math import pi, sin, cos, tan, sqrt

    _deg2rad = pi / 180.0
    _rad2deg = 180.0 / pi

    # WGS-84 Ellipsoid yields these criteria:
    a = 6378137
    eccSquared = 0.00669438
    k0 = 0.9996

    ZoneNumber = 16

    # Longitude must be between -180 & 179.9 (input in decimal degrees)
    LongTemp = (Long+180)-int((Long+180)/360)*360-180 # -180.00 .. 179.9
    LongRad = LongTemp * _deg2rad

    LongOrigin = (ZoneNumber - 1)*6-180+3 # +3 puts origin in middle of zone
    LongOriginRad = LongOrigin*_deg2rad

    LatRad = Lat * _deg2rad

    eccPrimeSquared = eccSquared/(1 - eccSquared)
    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad))
    T = (tan(LatRad))**2
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad)
    A = cos(LatRad)*(LongRad-LongOriginRad)
    M = a*((1
            - eccSquared/4
            - 3*eccSquared*eccSquared/64
            - 5*eccSquared*eccSquared*eccSquared/256)*LatRad 
           - (3*eccSquared/8
              + 3*eccSquared*eccSquared/32
              + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
           + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
           - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad))
    Easting = (k0*N*(A+(1-T+C)*A*A*A/6
                        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                  + 500000.0)
    Northing = (k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                                        + (61
                                           -58*T
                                           +T*T
                                           +600*C
                                           -330*eccPrimeSquared)*A*A*A*A*A*A/720)))
    return (Easting, Northing)


def survey(self):
    """
    reads "DecLat DecLon \n" text file and groups into 2 lists
    e.g., stripes = [[lat1, lon1], [lat2, lon2], ...]
    """
    stripes = []
    centers = []

    for stripe_file_loc in self.survey_stripe_locs:
        stripe_file = open(stripe_file_loc, 'rU')
        for line in stripe_file:
            line = line[0:-2] # remove the '\n' at the end
            pt_list = line.split(' ')
            stripes.append(pt_list)

    for center_file_loc in self.survey_center_locs:
        center_file = open(center_file_loc, 'rU')
        for line in center_file:
            line = line[0:-2] # remove the '\n' at the end
            pt_list = line.split(' ')
            centers.append(pt_list)

    return stripes, centers


def create_map(self):
    """
    This function collects the survey data and puts it into marker arrays ready to be published in rviz
    """
    from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
    import rospy

    # (stripe_inner, lane_inner, stripe_middle, lane_outer, stripe_outer) = survey(self)
    # stripes = [stripe_inner, stripe_middle, stripe_outer]
    # lanes = [lane_inner, lane_outer]
    (stripes, centers) = survey(self)
    
    self.map_stripe_array = MarkerArray()
    self.map_lane_array = MarkerArray()
    
    NCAT_id = 0
    
    ### Stripes ################################################################
    for pt in stripes:
        lat = float(pt[0])
        lon = float(pt[1])
        (east, nrth) = ll2utm(lat, lon) # convert to UTM
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.id = NCAT_id # enumerate subsequent markers here
        marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
        marker.lifetime = rospy.Duration() # will last forever unless modified
        marker.ns = "stripes"
        marker.type = Marker.CUBE
        marker.pose.position.x = east - self.UTMdatum['E']
        marker.pose.position.y = nrth - self.UTMdatum['N']
        marker.pose.position.z = -1.55 # zero is a novatel mount level
        marker.color.r = 255
        marker.color.g = 255
        marker.color.b = 0
        marker.color.a = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.7
        marker.mesh_use_embedded_materials = False
        self.map_stripe_array.markers.append(marker)
        
        NCAT_id += 1
    self.map_stripe_publisher.publish(self.map_stripe_array) # publish stripes as markers
    print('Stripe Markers have been printed') 
    del marker

    ### Lane Centers ###########################################################
    for pt in centers:
        lat = float(pt[0])
        lon = float(pt[1])
        (east, nrth) = ll2utm(lat, lon) # convert to UTM
        
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.id = NCAT_id # enumerate subsequent markers here
        marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
        marker.lifetime = rospy.Duration() # will last forever unless modified
        marker.ns = "lane_centers"
        marker.type = Marker.SPHERE
        marker.pose.position.x = east - self.UTMdatum['E']
        marker.pose.position.y = nrth - self.UTMdatum['N']
        marker.pose.position.z = -1.55 # zero is a novatel mount level
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 0.75
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.mesh_use_embedded_materials = False
        self.map_lane_array.markers.append(marker)
        
        NCAT_id += 1
    self.map_lane_publisher.publish(self.map_lane_array) # publish lane centers as markers
    print('Lane Center Markers have been printed')


def create_map_mesh(self):
    """Puts a blender mesh of the paement and stripes (continuous) into rviz"""
    from visualization_msgs.msg import Marker, MarkerArray
    import rospy
    marker_array = MarkerArray()

    ### Track Pavement Publisher ###############################################
    marker = Marker()
    marker.header.frame_id = 'odom' # publish in static frame
    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration() # immortal unless changed
    marker.ns = "track_mesh_pavement"
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_use_embedded_materials = False
    marker.mesh_resource = self.track_mesh_resource# wahoo
    marker.pose.position.x = 0# - self.UTMdatum['E']
    marker.pose.position.y = 0# - self.UTMdatum['N']
    marker.pose.position.z = -1.57
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.r = 0.3
    marker.color.g = 0.3
    marker.color.b = 0.3
    marker.color.a = 1.0
    marker_array.markers.append(marker)

    ### Lane Marking Publisher #################################################
    marker = Marker()
    marker.header.frame_id = 'odom' # publish in static frame
    marker.id = 1
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration() # immortal unless changed
    marker.ns = "track_mesh_lane_markings"
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_use_embedded_materials = False
    marker.mesh_resource = self.marking_mesh_resource # wahoo
    marker.pose.position.x = 0# - self.UTMdatum['E']
    marker.pose.position.y = 0# - self.UTMdatum['N']
    marker.pose.position.z = -1.57
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.r = 255
    marker.color.g = 255
    marker.color.b = 0
    marker.color.a = 1.0
    marker_array.markers.append(marker)

    self.track_mesh_publisher.publish(marker_array)