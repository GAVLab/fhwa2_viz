#!/usr/bin/env python

# Copyright (c) 2011, Ye Cheng, Dorian Scholz
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

import collections
import qt_gui.qt_binding_helper  # @UnusedImport
from QtCore import Slot
from QtGui import QWidget, QVBoxLayout, QSizePolicy

import matplotlib
if matplotlib.__version__ < '1.1.0':
    raise RuntimeError('A newer matplotlib is required (at least 1.1.0)')

try:
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    # work around bug in dateutil
    import sys
    import thread
    sys.modules['_thread'] = thread
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure

import numpy
from pprint import pprint as pp
from itertools import islice


class MatDataPlot(QWidget):
    class Canvas(FigureCanvas):
        """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
        def __init__(self, parent=None):
            fig = Figure()
            rect = .035, .15, .95, .8
            self.axes = fig.add_axes(rect)
            self.axes.grid(True, color='gray')
            super(MatDataPlot.Canvas, self).__init__(fig)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

            self.axes.tick_params(axis='both', which='major', labelsize=10)
            self.axes.tick_params(axis='both', which='minor', labelsize=8)   

            self.axes.set_axis_bgcolor('k')
            self.axes.grid(True, color='gray', alpha=.7, lw=2, ls=':')
            self.axes.set_xlabel('Time Since Initialization (s)', size=9)
            self.axes.set_ylabel('Error Magnitude (m)', size=9)
            
            self.disp_text = None

            # size_in = fig.get_size_inches()
            # fig.set_size_inches(size_in[0]*1.4, size_in[1])

    _colors = ((1, 0, 1), (0, 1, 1), (0.5, 0.24, 0), (.24, 0.5, 0.24), (1, 0.5, 0))

    def __init__(self, parent=None):
        super(MatDataPlot, self).__init__(parent)
        self._canvas = MatDataPlot.Canvas()
        vbox = QVBoxLayout()
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

        self._color_index = 0
        self._curves = {}

        self.keep_secs = 30
        self.scale_steps = 150
        self.ref_name = 'RTK'
        self.tgt_name = 'FHWA2 Combined'

    def add_curve(self, curve_id, data_x, data_y):
        data_x = collections.deque(data_x)
        data_y = collections.deque(data_y)
        color = self._colors[self._color_index % len(self._colors)]
        self._color_index += 1
        plot = self._canvas.axes.plot(data_x, data_y, linewidth=1, picker=5, color=color)[0]
        self._curves[curve_id] = (data_x, data_y, plot)

    def draw_plot(self):
        # self._canvas.axes.set_title(' '.join(['Ground Plane Error Magnitude of Sensor:', self.tgt_name, 'for Reference:', self.ref_name]), size=9)

        # Set axis bounds
        ymin = 0
        ymax = None
        xmax = xmin = 0
        for curve in self._curves.values():
            data_x, data_y, plot = curve
            if len(data_x) == 0:
                continue

            xmax = data_x[-1]
            # xmin = xmax - 5
            xmin = xmax - self.keep_secs


            # if ymin is None:
            #     ymin = min(data_y)
            #     ymax = max(data_y)
            # else:
            #     ymin = min(min(data_y), ymin)
            #     ymax = max(max(data_y), ymax)
            # print('mat_data_plot::data_x length: %i' % len(data_x))
            # print('mat_data_plot::data_y length: %i' % len(data_y))

            # Scale y axis to the number of steps desired
            if len(data_y) < self.scale_steps:
                lo_ind = 0
            else:
                lo_ind = len(data_y) - self.scale_steps
            hi_ind = len(data_y)

            if ymax is None:
                ymax = max(list(islice(data_y, lo_ind, hi_ind)))
            else:
                ymax = max(max(list(islice(data_y, lo_ind, hi_ind))), ymax)


            # pad the min/max
            delta = max(ymax - ymin, 0.1)
            ymin -= .1 * delta
            ymax += .1 * delta

            self._canvas.axes.set_xbound(lower=xmin, upper=xmax)
            self._canvas.axes.set_ybound(lower=ymin, upper=ymax)

        # Set plot data on current axes
        for curve in self._curves.values():
            data_x, data_y, plot = curve
            plot.set_data(numpy.array(data_x), numpy.array(data_y))

        # overlay the current value in the middle
        if 'data_y' in locals() and data_y:
            print_val = ' '.join(['{0:.3f}'.format(data_y[-1]), 'm'])
        else:
            print_val = 'Horizontal Error is Undefined'

        if self._canvas.disp_text:
            self._canvas.disp_text.remove()
        self._canvas.disp_text = self._canvas.axes.text(0.5, 0.5, print_val,
                alpha=0.7, color='0.65', size=50, lod=True, family='Ubuntu Mono',
                horizontalalignment='center',
                verticalalignment='center',
                transform=self._canvas.axes.transAxes)

        self._canvas.draw()

    @Slot(str, float)
    def update_value(self, curve_id, x, y):
        data_x, data_y, _ = self._curves[curve_id]
        data_x.extend(x)
        data_y.extend(y)

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            del self._curves[curve_id]
