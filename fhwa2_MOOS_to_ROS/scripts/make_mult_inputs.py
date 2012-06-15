#!/usr/bin/python
"""
This script file is for producing *.alogs as dummy measurments in lieu of data from:
	PennState
	DSRC
	SRI 

The data will be in one file
"""

import os, sys
from numpy.random import normal


def main(msg):
	msg = msg.split()
	msg1 = msg
	msg2 = msg
	msg3 = msg

	msg1[2] = 'gPennSt'
	msg2[2] = 'gSRI'
	msg3[2] = 'gDSRC'

	if ((msg[1] == 'zLat') or (msg[1] == 'zLong')):
		wiggle = 0.00005 # stdDev of gaussian noise added, in lat/lon decimal degrees ~.5m
		for value in [msg1[3], msg2[3], msg3[3]]:
			value = float(value)
			value += normal(value, wiggle, 1)
			value = str(value)
	elif ((msg[1] == 'zLatStdDev') or (msg[1] == 'zLongStdDev')):
		wiggle = .01 # stdDev of gaussian noise added, in m
		for value in [msg1[3], msg2[3], msg3[3]]:
			value = float(value)
			value += normal(value, wiggle, 1)
			value = str(value)
	elif msg[1] == 'zCourse':
		wiggle = 10 # stdDev of gaussian noise added, in deg
		for value in [msg1[3], msg2[3], msg3[3]]:
			value = float(value)
			value += normal(value, wiggle, 1)
			value = str(value)

	for message in [msg, msg1, msg2, msg3]:
		line = alog_manip.reconstructLine(message)
		tgt.write(line + '\n')


############################################################################

if __name__ == '__main__':
	
	sys.path.append('/home/gavlab/alog_manip/scripts') # alog_manip module
	sys.path.append('/home/gavlab/alog_manip/alogs') # fhwa2_Novatel.alog (file with only)
	sys.path.append('/home/gavlab/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS/scripts')
	sys.path.append('/home/gavlab/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS/alogs')

	# os.chdir('/home/gavlab/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS')
	import alog_manip

	curdir = os.getcwd()
	os.chdir('/')
	
	NovOnlyAlog = '/home/gavlab/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS/alogs/Long_onlyNovatel.alog'
	ResultAlog = '/home/gavlab/rgc0003rosws/fhwa2_viz/fhwa2_MOOS_to_ROS/alogs/Long_novatel_plus3.alog'

	src = open(NovOnlyAlog, 'rU')
	tgt = open(ResultAlog, 'w')
	tgt.write('%% This file created to simulate 3 measurment sources varying from the Novatel\n')
	
	import time;
	localtime = time.asctime( time.localtime(time.time()) )
	tgt.write("%% Author Date: "+ localtime + '\n') # timestamp it

	for msg in src:
		if '%%' in msg:
			tgt.write(msg) # keep the original header
		else:
			main(msg)

	os.chdir(curdir)