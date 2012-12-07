#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from qt_gui.qt_binding_helper import loadUi
from QtCore import Qt, QTimer, qWarning, Slot
from QtGui import QWidget, QApplication

import roslib
roslib.load_manifest('fhwa2_gui')
import rospy
# from rxtools.rosplot import ROSData
from rosplot import ROSData
from rostopic import get_topic_type

from .mat_data_plot import MatDataPlot
from rqt_gui_py.plugin import Plugin
from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common.topic_helpers import is_slot_numeric

from pprint import pprint as pp


class MatPlotWidget(QWidget):
    """The actual Widget """
    def __init__(self):
        super(MatPlotWidget, self).__init__()
        self.setObjectName('MatPlotWidget')

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MatPlot.ui')
        loadUi(ui_file, self, {'MatDataPlot': MatDataPlot})

        self.subscribe_topic_button.setEnabled(False)

        self._topic_completer = TopicCompleter(self.topic_edit)
        self.topic_edit.setCompleter(self._topic_completer)

        self._start_time = rospy.get_time()
        self._rosdata = {}

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        # init and start update timer for plot
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._update_plot_timer.start(40)

        # connect combobox
        self.comboBox.currentIndexChanged.connect(self.on_combo_box_changed)

        # params (colors)
        self.texts = {}
        self.data_plot._colors = {}
        for tag in rospy.get_param('/tags'):
            self.texts[tag] = rospy.get_param('/'+tag+'_text')
            color = [int(i) for i in list(rospy.get_param('/'+tag+'_color').split(','))]
            self.data_plot._colors[self.texts[tag]] = color

        # start with subscription to gps
        self.add_topic('/error_mags/rtk_ref/flt_tgt/mag_horiz')
           

    def update_plot(self):
        for topic_name, rosdata in self._rosdata.items():
            data_x, data_y = rosdata.next()
            self.data_plot.update_value(topic_name, data_x, data_y)
        self.data_plot.draw_plot()

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qWarning('MatPlot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            ros_topic_name = item.data(0, Qt.UserRole)
            if ros_topic_name == None:
                qWarning('MatPlot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return

        # get topic name
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))

        # check for numeric field type
        is_numeric, message = is_slot_numeric(topic_name)
        if is_numeric:
            event.acceptProposedAction()
        else:
            qWarning('MatPlot.dragEnterEvent(): rejecting: "%s"' % (message))

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.add_topic(topic_name)

    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics()

        is_numeric, message = is_slot_numeric(topic_name)
        self.subscribe_topic_button.setEnabled(is_numeric)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()))

    def add_topic(self, topic_name):
        if topic_name in self._rosdata:
            qWarning('MatPlot.add_topic(): topic already subscribed: %s' % topic_name)
            return

        self._rosdata[topic_name] = ROSData(topic_name, self._start_time)
        data_x, data_y = self._rosdata[topic_name].next()
        color = self.data_plot._colors[self.comboBox.currentText()]
        self.data_plot.add_curve(topic_name, data_x, data_y, color)

    @Slot()
    def on_clear_button_clicked(self):
        self.clean_up_subscribers()

    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        if checked:
            self._update_plot_timer.stop()
        else:
            self._update_plot_timer.start(40)

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

    @Slot(str)
    def on_combo_box_changed(self, text):
        print('In on_combo_box_changed')
        self.on_clear_button_clicked()
        self.data_plot.tgt_name = self.comboBox.currentText()

        if self.comboBox.currentText() == 'Filtered':
            self.add_topic('/error_mags/rtk_ref/flt_tgt/mag_horiz')
        elif self.comboBox.currentText() == 'GPS':
            self.add_topic('/error_mags/rtk_ref/gps_tgt/mag_horiz')
        elif self.comboBox.currentText() == 'Penn St':
            self.add_topic('/error_mags/rtk_ref/psu_tgt/mag_horiz')
        elif self.comboBox.currentText() == 'SRI':
            self.add_topic('/error_mags/rtk_ref/sri_tgt/mag_horiz')

        window_title = ' '.join(['Ground Plane Error Magnitude of Sensor:',
                                  self.comboBox.currentText(),
                                  'for Reference:',
                                  self.data_plot.ref_name])
        self.setWindowTitle(QApplication.translate("MatPlotWidget", 
            window_title, None, QApplication.UnicodeUTF8))


class MatPlot(Plugin):
    """rQt wrapper for the widget that makes it a plugin"""
    def __init__(self, context):
        super(MatPlot, self).__init__(context)
        self.setObjectName('MatPlot')

        self._widget = MatPlotWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def close_plugin(self):
        self._widget.clean_up_subscribers()
