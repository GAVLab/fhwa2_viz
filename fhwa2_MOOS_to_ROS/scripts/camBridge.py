#!/usr/bin/env python
"""
displays camera data in rviz

Author: Robert Cofield
Created 7/20/2012, for GAVLab
"""
from yaml import load
import sys


class CAM2RVIZ(object):
    def __init__(self, config):
        object.__init__()


################################################################################
def main():
    config_filename = sys.argv[1]
    if config_filename[-4:] is not 'yaml':
        raise Exception('YAML configuration file required')
    stream = file(config_filename, 'r')
    this_config = load(stream)
    cam_app = CAM2RVIZ(this_config)


if __name__ == "__main__":
    main()