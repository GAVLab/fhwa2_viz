#!/usr/bin/python
"""
This script used to interpolate msgs from generated dummy inputs, creating msgs at ~100Hz
"""

import os, sys
sys.path.append('/home/gavlab/alog_manip/scripts')
import alog_manip as manip
# from numpy import interp
from scipy import interpolate
from pprint import pprint


loHzAlog = '/home/gavlab/alog_Files/Long_novatel_plus3.alog'
loHzDict = manip.alogrd_dict(loHzAlog)
desHz = 100
hiHzDict = manip.increaseFreq(loHzDict, desHz)

print('\n\n\n\n\nhiHzDict: ')
# pprint(hiHzDict)