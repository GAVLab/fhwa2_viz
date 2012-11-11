#!/usr/bin/env python
"""
assume that survey files are in the format:
    Lat Lon Alt (decimal degrees)
    space separated
    no header, or trailing spaces

Output will set altitude to zero (conform to GPS constraints)

output *.ply
"""
import sys, os
from util import GPS


in_locs = [
    '/home/rgcofield/devel/fhwa2_ws/fhwa2_viz/fhwa2_survey/survey/centers/TurnerFairbanks_Centerline.txt',
    '/home/rgcofield/devel/fhwa2_ws/fhwa2_viz/fhwa2_survey/survey/stripes/TurnerFairbanks_RoadEdge.txt']
out_loc = '/home/rgcofield/devel/fhwa2_ws/fhwa2_viz/fhwa2_survey/mesh/TF_points.ply'

gps = GPS()
datum_e = 313617.45721616584 +125
datum_n = 4314077.05485446 -50


def read_in(files):
    pts = []
    for f in files:
        for l in f:
            pts.append( [float(x) for x in l[0:-2].split(' ')] )
    return pts


def convert(pts):
    out = []
    for p in pts:
        (E, N, _), _ = gps.lla2utm(tuple(p))
        out.append((E-datum_e, N-datum_n, 0))
    return out


def make_header(pts):
    return '\n'.join(['ply',
                      'format ascii 1.0',
                      'element vertex '+str(len(pts)),
                      'property double x',
                      'property double y',
                      'property double z',
                      'end_header'])

 
def make_body(pts):
    return '\n'.join([' '.join([str(x) for x in p]) for p in pts])


def main():
    files = [open(l, 'rU') for l in in_locs]
    
    pts_utm = convert(read_in(files))
    
    f = open(out_loc, 'w')
    f.write('\n'.join( [make_header(pts_utm), make_body(pts_utm)] ))

    f.close()


if __name__ == '__main__':
    main()