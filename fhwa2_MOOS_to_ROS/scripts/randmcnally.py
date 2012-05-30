#!/usr/bin/env python
"""
This file contain modules related to coordinate systems, and specifically the NCAT track map.


Created 5/20/2012
Author: Robert Cofield
"""

def survey():

    from csv import reader as rdr

    ###
    stripe_inner_lat_rd = rdr(open("stripe_inner_lat.txt"), delimiter=" ")
    stripe_inner_lat = []
    for col in stripe_inner_lat_rd:
        stripe_inner_lat.append(col)
    del stripe_inner_lat[0][0]
    stripe_inner_lat = stripe_inner_lat[0]
    stripe_inner_lon_rd = rdr(open("stripe_inner_lon.txt"), delimiter=" ")
    stripe_inner_lon = []
    for col in stripe_inner_lon_rd:
        stripe_inner_lon.append(col)
    del stripe_inner_lon[0][0]
    stripe_inner_lon = stripe_inner_lon[0]

    stripe_inner = {}
    for pt in range(len(stripe_inner_lon)):
        stripe_inner[pt] = [stripe_inner_lat[pt], stripe_inner_lon[pt]]

    ###
    lane_inner_lat_rd = rdr(open("lane_inner_lat.txt"), delimiter=" ")
    lane_inner_lat = []
    for col in lane_inner_lat_rd:
        lane_inner_lat.append(col)
    del lane_inner_lat[0][0]
    lane_inner_lat = lane_inner_lat[0]
    lane_inner_lon_rd = rdr(open("lane_inner_lon.txt"), delimiter=" ")
    lane_inner_lon = []
    for col in lane_inner_lon_rd:
        lane_inner_lon.append(col)
    del lane_inner_lon[0][0]
    lane_inner_lon = lane_inner_lon[0]

    lane_inner = {}
    for pt in range(len(lane_inner_lon)):
        lane_inner[pt] = [lane_inner_lat[pt], lane_inner_lon[pt]]

    ###
    stripe_middle_lat_rd = rdr(open("stripe_middle_lat.txt"), delimiter=" ")
    stripe_middle_lat = []
    for col in stripe_middle_lat_rd:
        stripe_middle_lat.append(col)
    del stripe_middle_lat[0][0]
    stripe_middle_lat = stripe_middle_lat[0]
    stripe_middle_lon_rd = rdr(open("stripe_middle_lon.txt"), delimiter=" ")
    stripe_middle_lon = []
    for col in stripe_middle_lon_rd:
        stripe_middle_lon.append(col)
    del stripe_middle_lon[0][0]
    stripe_middle_lon = stripe_middle_lon[0]

    stripe_middle = {}
    for pt in range(len(stripe_middle_lon)):
        stripe_middle[pt] = [stripe_middle_lat[pt], stripe_middle_lon[pt]]

    ###
    lane_outer_lat_rd = rdr(open("lane_outer_lat.txt"), delimiter=" ")
    lane_outer_lat = []
    for col in lane_outer_lat_rd:
        lane_outer_lat.append(col)
    del lane_outer_lat[0][0]
    lane_outer_lat = lane_outer_lat[0]
    lane_outer_lon_rd = rdr(open("lane_outer_lon.txt"), delimiter=" ")
    lane_outer_lon = []
    for col in lane_outer_lon_rd:
        lane_outer_lon.append(col)
    del lane_outer_lon[0][0]
    lane_outer_lon = lane_outer_lon[0]

    lane_outer = {}
    for pt in range(len(lane_outer_lon)):
        lane_outer[pt] = [lane_outer_lat[pt], lane_outer_lon[pt]]

    ###
    stripe_outer_lat_rd = rdr(open("stripe_outer_lat.txt"), delimiter=" ")
    stripe_outer_lat = []
    for col in stripe_outer_lat_rd:
        stripe_outer_lat.append(col)
    del stripe_outer_lat[0][0]
    stripe_outer_lat = stripe_outer_lat[0]
    stripe_outer_lon_rd = rdr(open("stripe_outer_lon.txt"), delimiter=" ")
    stripe_outer_lon = []
    for col in stripe_outer_lon_rd:
        stripe_outer_lon.append(col)
    del stripe_outer_lon[0][0]
    stripe_outer_lon = stripe_outer_lon[0]

    stripe_outer = {}
    for pt in range(len(stripe_outer_lon)):
        stripe_outer[pt] = [stripe_outer_lat[pt], stripe_outer_lon[pt]]

	return (stripe_inner, lane_inner, stripe_middle, lane_outer, stripe_outer)



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

	# not quite sure what else to do here


from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
import rospy
def create_map(self):
        (stripe_inner, lane_inner, stripe_middle, lane_outer, stripe_outer) = survey()
        stripes = [stripe_inner, stripe_middle, stripe_outer]
        lanes = [lane_inner, lane_outer]
        self.map_stripe_array = MarkerArray()
        self.map_lane_array = MarkerArray()
        NCAT_id = 0
        
        # Stripes
        for ring in stripes:
            for pt in ring:
                lat = float(ring[pt][0])
                lon = float(ring[pt][1])
                (east, nrth) = ll2utm(lat, lon) # convert to UTM
                
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.id = NCAT_id # enumerate subsequent markers here
                marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
                marker.lifetime = rospy.Duration() # will last forever unless modified
                marker.ns = "stripes"
                marker.type = Marker.CUBE
                marker.pose.position.x = east
                marker.pose.position.y = nrth
                marker.color.r = 1
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1.0
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.75
                marker.mesh_use_embedded_materials = False

                self.map_stripe_array.markers.append(marker)
                
                print('Stripes')
                print(marker)
                NCAT_id += 1

        # Centers of the lanes
        for ring in lanes:
            for pt in ring:
                lat = float(ring[pt][0])
                lon = float(ring[pt][1])
                (east, nrth) = ll2utm(lat, lon) # convert to UTM
                
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.id = NCAT_id # enumerate subsequent markers here
                marker.action = Marker.ADD # can be ADD, REMOVE, or MODIFY
                marker.lifetime = rospy.Duration() # will last forever unless modified
                marker.ns = "lane_centers"
                marker.type = Marker.SPHERE
                marker.pose.position.x = east
                marker.pose.position.y = nrth
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.mesh_use_embedded_materials = False
                
                self.map_lane_array.markers.append(marker)
                
                print('Lane Centers')
                print(marker)
                NCAT_id += 1
                