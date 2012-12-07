#!/usr/bin/env python
import rospy, roslib
roslib.load_manifest('fhwa2_gui')
from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg
from fhwa2_gui.msg import ECEFpos
from std_msgs.msg._Int8 import Int8

from pprint import pprint as pp 

class SPLITTER(object):

    def __init__(self):
        super(SPLITTER, self).__init__()
        self.DEBUG = bool(rospy.get_param('~DEBUG'))
        
        self.sub_topic = '/moos/' + rospy.get_param('~moosapp_name')
        self.sub = rospy.Subscriber(self.sub_topic, MOOSrosmsg, self.callback)

        self.pubs = {}
        self.tags = rospy.get_param('/tags')
        self.n_solns = len(self.tags)
        for tag in self.tags:
            self.pubs[tag] = rospy.Publisher('/moos/solutions/'+tag, ECEFpos)

        self.orient_index = int(rospy.get_param('~orient_index'))        
        self.numsat_index = int(rospy.get_param('~numsat_index'))

        if self.DEBUG:
            # print('splitter -\n\t# of solutions: %i' % self.n_solns)
            # print('\torient_index: %i' % self.orient_index)
            # print('\tnumsat_index: %i' % self.numsat_index)
            pass
            
        self.pubs['numsat'] = rospy.Publisher('/moos/numsat', Int8)


    def callback(self, msg):
        if self.DEBUG:
            print('splitter callback: incoming msg: \n\t%s' % msg.MOOSstring)
        msgs = {}
        vals = msg.MOOSstring.split('{')[1].strip('}').split(',')
        n_left_over = len(vals) % 6
        orientation = vals[self.orient_index]
        
        for n in range(self.n_solns):
            msgs[self.tags[n]] = vals[(n*6):((n+1)*6)]
            msgs[self.tags[n]].append(orientation)
            msgs[self.tags[n]].append(msg.header.stamp)

        msgs['numsat'] = int(vals[self.numsat_index])

        if self.DEBUG:
            print('\nsplitter callback - dict of output messages:')
            pp(msgs)
        # ship off solutions
        self.disseminate(msgs)


    def disseminate(self, msgs):
        for tag in self.tags:
            if '-nan' in msgs[tag]:
                continue
            msg = ECEFpos()
            msg.x = float(msgs[tag][0])
            msg.y = float(msgs[tag][1])
            msg.z = float(msgs[tag][2])
            msg.x_covar = float(msgs[tag][3])
            msg.y_covar = float(msgs[tag][4])
            msg.z_covar = float(msgs[tag][5])
            msg.orient = float(msgs[tag][6])
            msg.header.stamp = msgs[tag][7]

            self.pubs[tag].publish(msg)

        msg = Int8()
        msg.data = msgs['numsat']
        try:
            self.pubs['numsat'].publish(msg)
        except Exception, e:
            print('\n\nslitter: cannot publish numsat msg -')
            print e


def main():
    rospy.init_node('splitter')
    app = SPLITTER()
    rospy.spin()


if __name__ == '__main__':
    main()