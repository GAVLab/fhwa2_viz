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

upd_hi = False
if upd_hi:
    desHz = 100
    alog = MOOSalog(lo_file, hi_file)
    alog.increaseFreq(desHz)
    alog.makeChronList()
    alog.writeChronListToFile()
    alog.closefiles()

## Examine output
import matplotlib.pyplot as plt

alog = MOOSalog(hi_file)

crs = []
crs_t = sorted(alog.srcData['gNovatel']['zCourse'])
for t in crs_t: # using iteritems may not be in chronological order
    crs.append(alog.srcData['gNovatel']['zCourse'][t])
fig = plt.figure(1)
plt.plot(crs_t, crs)
plt.title("Course (deg)")
plt.show(1)

lat = []
lat_t = sorted(alog.srcData['gNovatel']['zLat'])
for t in lat_t:
    lat.append(alog.srcData['gNovatel']['zLat'][t])
fig = plt.figure(2)
plt.plot(lat_t, lat)
plt.title("Latitude (dec. deg.)")
plt.show(2)

lon = []
lon_t = sorted(alog.srcData['gNovatel']['zLong'])
for t in lon_t:
    lon.append(alog.srcData['gNovatel']['zLong'][t])
fig = plt.figure(3)
plt.plot(lon_t, lon)
plt.title("Longitude (dec. deg.)")
plt.show()

latstd = []
latstd_t = sorted(alog.srcData['gNovatel']['zLatStdDev'])
for t in latstd_t:
    latstd.append(alog.srcData['gNovatel']['zLatStdDev'][t])
fig = plt.figure()
plt.plot(latstd_t, latstd)
plt.title("Lat Std Dev (m)")
plt.show()

lonstd = []
lonstd_t = sorted(alog.srcData['gNovatel']['zLongStdDev'])
for t in lonstd_t:
    lonstd.append(alog.srcData['gNovatel']['zLongStdDev'][t])
fig = plt.figure()
plt.plot(lonstd_t, lonstd)
plt.title("Long Std Dev")
plt.show()