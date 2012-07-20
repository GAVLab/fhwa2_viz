#!/usr/bin/env python
"""
This class puts a map of markers and meshes in rViz
It does not connect to MOOS
"""
import sys
from yaml import load
import mapping # fhwa2 module
import roslib; roslib.load_manifest('fhwa2_MOOS_to_ROS')
import rospy
from visualization_msgs.msg import Marker, MarkerArray # had to add module to manifest


class MAP2RVIZ(object):
    def __init__(self, config):
        object.__init__(self)
        self.get_config(config)
        self.set_publishers()
        mapping.create_map(self) # create marker arrays of the stripes and lane centers
        print('Map markers published - you should see the lane/stripe markers once running')
        mapping.create_map_mesh(self)
        print('Map mesh has been published - you should see it once running')


    def get_config(self, config):
        self.UTMdatum = config["UTMdatum"] # dict
        self.survey_stripe_locs = config["survey_stripe_locs"]
        self.survey_center_locs = config["survey_center_locs"]
        self.track_mesh_resource = config["track_mesh_resource"]
        self.marking_mesh_resource = config["marking_mesh_resource"]


    def set_publishers(self):
        self.map_stripe_publisher = rospy.Publisher('/map/survey_stripes', MarkerArray, latch=True)
        self.map_lane_publisher = rospy.Publisher('/map/survey_lanes', MarkerArray, latch=True)
        self.track_mesh_publisher = rospy.Publisher('/map/mesh', MarkerArray, latch=True)


def main():
    #setup ROS node
    rospy.init_node('moos2rviz_survey')

    ## setup config file
    config_file = sys.argv[1]
    if config_file[-4:] != 'yaml': # check file format
        print("Config file must be YAML format!!! That's how we do.")
    stream = file(config_file,'r')
    this_config = load(stream) # loads as a dictionary

    #Setup Map App
    app = MAP2RVIZ(this_config)
    
    #Setup ROS Stuff
   
    #spin
    rospy.spin()


if __name__ == '__main__':
    main()