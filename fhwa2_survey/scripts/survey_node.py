#!/usr/bin/env python
"""
This class puts a map of markers and meshes in rViz

Outputs in UTM coordinates

If a file or multiple files are input, it will output a marker array.
If a position is input via roslaunch param, it will output a single marker

Author: Robert Cofield, for GAVLab, 12/5/2012
"""
import sys, os
from yaml import load
import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest
from math import pi, sin, cos, tan, sqrt
from util import GPS
from pprint import pprint as pp
from copy import deepcopy as dcp


def read_survey(file_locs, default, delimiter=', '):
    """ file_locs is the absolute location of the files 
        delimiter is self explanatory
        default is the position value to set when 2D instead of 3D (outputs 3D regardless)
    """
    print('\n\ndelimiter: __|%s|__\n\n' % delimiter)
    out = []
    files = [open(f, 'rU') for f in file_locs]
    for f in files:
        for line in f:
            # data = [float(i) for i in line[0:-2].split(delimiter) if line[0:-2]]
            line = line[0:-2] # remove the '\n' at the end
            pt_list = line.split(' ')
            pt_list = [float(i) for i in pt_list]
            out.append(pt_list)

            if len(pt_list) == 2:
                pt_list.append(default) 
            elif len(pt_list) != 3:
                print('read_survey: Warning: incorrect number of elements. skipping line.')
                continue

            # point = (default, default, default)
            # for n in range(len(pt_list)):
            #     point[n] = pt_list[n]
            # out.append(point)

    return out


class SURVEY(object):
    def __init__(self):
        object.__init__(self)
        self.gps = GPS()

        self.UTMdatum = rospy.get_param('/UTMdatum')
        self.coord_sys_input = rospy.get_param('~coord_sys_input')

        # display parameters
        try:
            self.scale = [float(i) for i in rospy.get_param('~scale').split(', ')]
        except:
            self.scale = [1, 1, 1]
        self.rgba = [float(i) for i in rospy.get_param('~rgba').split(', ')]

        # create the base marker
        self.marker = Marker()
        self.marker.header.frame_id = rospy.get_param('~frame_id')
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration()
        self.marker.ns = 'fhwa2_survey'
        self.marker.scale.x = self.scale[0]
        self.marker.scale.y = self.scale[1]
        self.marker.scale.z = self.scale[2]
        self.marker.color.r = self.rgba[0]
        self.marker.color.g = self.rgba[1]
        self.marker.color.b = self.rgba[2]
        self.marker.color.a = self.rgba[3]

        m_type = rospy.get_param('~marker')
        if m_type == 'CUBE':
            self.marker.type = Marker.CUBE
        elif m_type == 'SPHERE':
            self.marker.type = Marker.SPHERE
        elif m_type[0:4] == 'file':
            self.marker.type = Marker.MESH_RESOURCE
            self.marker.mesh_use_embedded_materials = False
            self.marker.mesh_resource = m_type
        else:
            print('invalid type for marker given')
            return

        try: # see if the positions are in a file
            self.files = rospy.get_param('~file_locs').split(', ')
            self.position = read_survey(self.files, 0, delimiter=rospy.get_param('~delimiter')) #TODO make default more robust
            self.convert_data(mult=True)
            self.publish_array()
        except KeyError:
            self.position = [float(i) for i in rospy.get_param('~position').split(', ')]
            self.convert_data()
            self.publish()


    def convert_data(self, mult=False):
        """figure out how to get data into UTM"""
        # Decide which function is needed
        if self.coord_sys_input == 'LLA':
            convert = self.gps.lla2utm
        else:
            print('survey_node: Unknown or invalid input coordinate system')
            return

        if mult:
            for p in range(len(self.position)):
                (E, N, _), _ = convert(self.position[p])
                self.position[p] = (E, N, 0)
        else:
            (E, N, _), _ = convert((self.position[0], self.position[1], self.position[2]))
            self.position = (E, N, 0)


    def publish_array(self):
        pub = rospy.Publisher('survey', MarkerArray, latch=True)
        array = MarkerArray()
        m_id = 0
        for p in self.position:
            m = dcp(self.marker)
            m.id = m_id
            m.pose.position.x = p[0] - float(self.UTMdatum['E'])
            m.pose.position.y = p[1] - float(self.UTMdatum['N'])
            m.pose.position.z = p[2]
            array.markers.append(m)
            m_id += 1
        pub.publish(array)


    def publish(self):
        pub = rospy.Publisher('survey', Marker, latch=True)
        m = dcp(self.marker)
        m.id = 0
        m.pose.position.x = self.position[0] - float(self.UTMdatum['E'])
        m.pose.position.y = self.position[1] - float(self.UTMdatum['N'])
        m.pose.position.z = self.position[2]
        pub.publish(m)


def main():
    rospy.init_node('moos2rviz_survey')
    app = SURVEY()
    rospy.spin()

if __name__ == '__main__':
    main()