#!/usr/bin/env python
import os

from qt_gui.qt_binding_helper import loadUi
from QtCore import Qt
from QtGui import QWidget

from palette.red import *
from palette.yellow import *
from palette.white import *

import roslib
roslib.load_manifest('fhwa2_gui')
roslib.load_manifest('fhwa2_MOOS_to_ROS')
from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg
import rospy
from std_msgs.msg._Int8 import Int8


from rqt_gui_py.plugin import Plugin


class NumSatWidget(QWidget):
    """docstring for """
    def __init__(self):
        super(NumSatWidget, self).__init__()
        self.setObjectName('NumSatWidget')

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'NumSat.ui')
        loadUi(ui_file, self)
        self.sub = rospy.Subscriber('/moos/numsat', Int8, self.onUpd)
        self.palette_is_normal = True

    def onUpd(self, msg):
        val = msg.data
        self.NumLcd.display(val)


class NumSat(Plugin):
    """docstring"""
    def __init__(self, context):
        super(NumSat, self).__init__(context)
        self.setObjectName('NumSat')

        self._widget = NumSatWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def close_plugin(self):
        print 'unsubscribe here'
