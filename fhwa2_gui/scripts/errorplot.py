#!/usr/bin/env python
import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

class ErrorPlot(FigureCanvas):
    """This is a widget that will plot the desired error value (ref to RTK?"""
    def __init__(self, parent=None):
        self.figure = Figure()
        super(ErrorPlot, self).__init__(self.figure)
        self.clear()

    def clear(self):
        self.line, 