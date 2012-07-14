#!/usr/bin/env python
"""
This script used to interpolate msgs from generated dummy inputs, creating msgs at ~100Hz
"""

import sys
sys.path.append('/home/gavlab/')
from alog_manip.MOOSalog import MOOSalog
from pprint import pprint

lo_file = '/home/gavlab/alog_Files/Long_novatel_plus3.alog'
hi_file = '/home/gavlab/alog_Files/Long_novatel_plus3_100Hz.alog'
desHz = 100
alog = MOOSalog(lo_file, hi_file)
alog.increaseFreq(desHz)

# print('\n\n\nhiHzDict: ')
