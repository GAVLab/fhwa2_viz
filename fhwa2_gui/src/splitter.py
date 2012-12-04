#!/usr/bin/env python
import rospy, roslib
roslib.load_manifest('fhwa2_gui')
from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg
from fhwa2_gui.msg import ECEFpos


class SPLITTER(object):

    def __init__(self):
        super(SPLITTER, self).__init__()
        
        self.sub_topic = '/moos/' + rospy.get_param('~moosapp_name')
        self.sub = rospy.Subscriber(self.sub_topic, MOOSrosmsg, self.callback)
        self.pubs = {}

        self.tags = []

        # configs
        def get_solution_(self, n):
            s = '~soln' + str(n)
            tag = rospy.get_param(s + '_tag')
            self.tags.append(tag)

            pub = rospy.Publisher('/moos/solutions/'+tag, ECEFpos)
            self.pubs[tag] = pub 

        n = 0
        while True:
            try:
                get_solution_(n)
            except:
                break
            n += 1
        self.n_solns = n
        self.orient_index = int(rospy.get_param('~orient_index'))
        self.numsat_index = int(rospy.get_param('~numsat_index'))

    def callback(self, msg):        
        msgs = {}
        vals = msg.MOOSstring.split('{')[1].strip('}').split(',')
        n_left_over = len(vals) % 6
        orientation = vals[self.orient_index]
        
        for n in range(self.n_solns):
            msgs[self.tags[n]] = vals[(n*6):((n+1)*6)]
            msgs[self.tags[n]].append(orientation)
            
        # ship off solutions
        self.disseminate(msgs)

    def disseminate(self, msgs):
        for tag in msgs:
            msg = ECEFpos()
            msg.x = float(msgs[tag][0])
            msg.y = float(msgs[tag][1])
            msg.z = float(msgs[tag][2])
            msg.x_covar = float(msgs[tag][3])
            msg.y_covar = float(msgs[tag][4])
            msg.z_covar = float(msgs[tag][5])
            msg.orient = float(msgs[tag][6])

            self.pubs[tag].publish(msg)


def main():
    rospy.init_node('splitter')
    app = SPLITTER()
    rospy.spin()


if __name__ == '__main__':
    main()