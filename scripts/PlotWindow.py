#!/usr/bin/env python
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as  NavigationToolbar
from matplotlib.figure import Figure

class PlotWindow(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('Sliding histogramm')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        self.axes_1.clear()
        self.axes_1.grid(True)

        self.axes_2.clear()
        self.axes_2.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((10.0, 6.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes_1 = self.fig.add_subplot(311)
        self.axes_2 = self.fig.add_subplot(312)
        self.axes_3 = self.fig.add_subplot(313)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        self.pauseButton = QPushButton('Pause', self)
        self.resetButton = QPushButton('Reset', self)
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        hbox = QHBoxLayout()
        vbox.addLayout(hbox)
        hbox.addWidget(self.pauseButton)
        hbox.addWidget(self.resetButton)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)
