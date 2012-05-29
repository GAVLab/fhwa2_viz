# -*- coding: utf-8 -*-
"""
Created on Mon May 28 15:36:16 2012

@author: Robert cofield
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