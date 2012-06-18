#!/usr/bin/python
"""
This script file is for producing *.alogs as dummy measurments in lieu of data from:
	PennState
	DSRC
	SRI 

The data will be in one file, all alogs should be stored on all machines in /home/__user_name__/alog_Files/

"""

import os, sys
from numpy.random import normal


def main(msg):
	msg = msg.split()
	msgs = [msg[:], msg[:], msg[:], msg[:]]

	msgs[1][2] = 'gPennSt'
	msgs[2][2] = 'gSRI'
	msgs[3][2] = 'gDSRC'

	if ((msg[1] == 'zLat') or (msg[1] == 'zLong')):
		wiggle = 0.00005 # stdDev of gaussian noise added, in lat/lon decimal degrees ~.5m
	elif ((msg[1] == 'zLatStdDev') or (msg[1] == 'zLongStdDev')):
		wiggle = .01 # stdDev of gaussian noise added, in m
	elif msg[1] == 'zCourse':
		wiggle = 10 # stdDev of gaussian noise added, in deg
		
	spreadOut(msgs, wiggle)
	for message in msgs:
		line = alog_manip.reconstructLine(message)
		tgt.write(line + '\n')


def spreadOut(msgs, wiggle):
	noises = normal(msgs[0][3], wiggle, (len(msgs)-1) )
	for ind in [1,2,3]: # only modify values of 3 dummy msgs
		value = noises[ind-1]
		msgs[ind][3] = str(value)
		if 'StdDev' in msgs[ind][1]: # cannot have a negative std dev
			msgs[ind][3] = str(abs(value))


############################################################################

if __name__ == '__main__':
	
	sys.path.append('/home/gavlab/alog_manip/scripts') # alog_manip module
	sys.path.append('/home/gavlab/alog_manip/alogs') # fhwa2_Novatel.alog (file with only)
	sys.path.append('/home/gavlab/fhwa2_ws/fhwa2_viz/fhwa2_MOOS_to_ROS/scripts') # where this is
	sys.path.append('/home/gavlab/alog_Files') # location of alog files

	# os.chdir('/home/gavlab/fhwa2_ws/fhwa2_viz/fhwa2_MOOS_to_ROS')
	import alog_manip

	curdir = os.getcwd()
	os.chdir('/')
	
	NovOnlyAlog = '/home/gavlab/alog_Files/Long_onlyNovatel.alog'
	ResultAlog = '/home/gavlab/alog_Files/Long_novatel_plus3.alog'
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

	print('Writing complete open target *.alog file to confirm')
	os.chdir(curdir)