#!/usr/bin/env python
import os

from qt_gui.qt_binding_helper import loadUi
from QtCore import Qt
from QtGui import QWidget

import roslib
roslib.load_manifest('fhwa2_gui')
roslib.load_manifest('fhwa2_MOOS_to_ROS')
from fhwa2_MOOS_to_ROS.msg import MOOSrosmsg
import rospy

from rqt_gui_py.plugin import Plugin


class NumSatWidget(QWidget):
    """docstring for """
    def __init__(self):
        super(NumSatWidget, self).__init__()
        self.setObjectName('NumSatWidget')

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'NumSat.ui')
        loadUi(ui_file, self)
        self.sub = rospy.Subscriber('/moos/gMOOS2ROS', MOOSrosmsg, self.onUpd)

    def onUpd(self, msg):
        if msg.MOOSname == 'zpsrNumObs':
            self.NumLcd.display(int(msg.MOOSdouble))


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
