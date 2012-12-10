#!/usr/bin/env python
import sys, os
from util import GPS

in_loc =  '/home/rgcofield/devel/fhwa2_ws/fhwa2_viz/fhwa2_survey/survey/signs/sections_ecef.txt'
out_loc = '/home/rgcofield/devel/fhwa2_ws/fhwa2_viz/fhwa2_survey/survey/signs/sections_utm.txt'
out_file = open(out_loc, 'w')

gps = GPS()
datum_e = 659300
datum_n = 3607850

for line in open(in_loc, 'rU'):
    pt = line[0:-1].split(' ')
    print(pt)

    if pt[0] == '':
        out_file.write('\n')
        continue
    if pt[-1][0] not in ['n','s','e','w']: # exclude stop/yield/etc
        out_file.write('\n')
        continue

    (E, N, _), _ = gps.ecef2utm((float(pt[0]), float(pt[1]), float(pt[2])))
    line = ', '.join([str(E-datum_e), str(N-datum_n), '0'])
    out_file.write(line+'\n')

out_file.close()