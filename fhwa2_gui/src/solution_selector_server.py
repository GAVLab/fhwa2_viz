#!/usr/bin/env python
import roslib
roslib.load_manifest('fhwa2_gui')
import rospy
from fhwa2_gui.srv import *


class SolutionSelectServer(object):
    """ Receives request string of what to have turned on from a selector widget
        and calls publish enablement services for all solutions with the 
        respective state"""

    def __init__(self):
        super(SolutionSelectServer, self).__init__()
        self.svc = rospy.Service('solution_selector_master', SolutionSelect, self.callback)

        # Get tags
        self.reference = rospy.get_param('~reference_tag')
        self.tags = rospy.get_param('~tags').split(', ')

        self.selector_subservice_names = []
        for t in self.tags:
            self.selector_subservice_names.append(t)

        # Create the service proxies
        self.selector_subservices = {}
        for n in self.selector_subservice_names:
            self.selector_subservices[n] = rospy.ServiceProxy( \
                'solution_selector_'+n, SolutionEnable)
            
    def callback(self, req):
        """ Assume the reference topic will always be enabled
            Doesn't support multi-target selection *yet* """
        # Turn on desired target
        if not req.target: # allow for turning all solutions aff
            pass
        elif req.target in self.tags: # ensure service exists
            print('SolutionSelectServer: calling   %s   to turn on' % req.target)
            _ = self.selector_subservices[req.target](True)

        # Turn off everything else - this will be none for single solution set
        tag_gen = [n for n in self.tags if n != req.target]
        if len(tag_gen):
            for t in tag_gen:
                print('SolutionSelectServer: calling %s to turn off' % t)
                _  = self.selector_subservices[t](False)


        return True


def main():
    rospy.init_node('solution_selector')
    srv = SolutionSelectServer()
    rospy.spin()


if __name__ == '__main__':
    main()