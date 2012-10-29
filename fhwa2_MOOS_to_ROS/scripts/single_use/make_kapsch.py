#!/usr/bin/env python
"""
change dsrc to kapsch in alogs
"""
import sys, os
import copy.deepcopy as dcp
sys.path.append('/home/gavlab/devel')
from alog_manip.python import MOOSalog

alogSrc = '/home/gavlab/devel/alog_Files/Long_novatel_plus3.alog'
alogDst = '/home/gavlab/devel/alog_Files/Long_novatel_plus3_rename_kapsch.alog'

tgtStr = ['gDSRC']
desStr = ['gKapsch']

replStr(alogSrc, alogDst, tgtStr, desStr)