#!/usr/bin/env python
"""
Mathematical functions related to coordinates and transforms
"""

from math import pi, sqrt, sin, cos, tan

def ll2utm(Lat, Long):
    """
    Converts decimal degree latitude & longitude to UTM Northing & Easting

    Adaptation of code from http://mahi.ucsd.edu/class233/data/Basic/LatLongUTMconversion.py
    Author: Robert Cofield
    20 May, 2012
    Only works for UTM Zone 16 (any letter designation), Northern Hemisphere
    must stay in same letter designation.
    """
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