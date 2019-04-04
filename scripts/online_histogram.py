#!/usr/bin/env python

import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from reef_msgs.msg import SyncEstimateError
import math
from collections import deque
import matplotlib.pyplot as plt
from scipy.stats import norm
import matplotlib.mlab as mlab
import numpy as np
#!/usr/bin/env python
from PlotWindow import PlotWindow

import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *


class verifyRGBD(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)

        print("Verifying RGBD")

        rospy.Subscriber("estimate_error", SyncEstimateError, self.error_cb)

        self.window_size = 10000
        self.counter = 0
        self.index=0
        self.x_deque= deque(maxlen=self.window_size)
        self.y_deque= deque(maxlen=self.window_size)
        self.z_deque= deque(maxlen=self.window_size)
        self.paused = False

        self.pauseButton.clicked.connect(self.pauseClicked)
        self.resetButton.clicked.connect(self.resetClicked)

    def pauseClicked(self):
        if self.paused:
            self.paused = False
        else:
            self.paused = True

    def resetClicked(self):
        self.draw_counter =0
        self.x_deque.clear()
        self.y_deque.clear()
        self.index=0
        self.paused = False

    def error_cb(self, error_msg):

        sd_x = error_msg.velocitySDPlus.x/3
        sd_y = error_msg.velocitySDPlus.y/3
        sd_z = error_msg.zSDPlus/3

        std_x_error = error_msg.velocityError.x/sd_x
        std_y_error = error_msg.velocityError.y/sd_y
        std_z_error = error_msg.zError/sd_z

        if not np.isnan(std_x_error):
            self.x_deque.append(round(std_x_error,3))
        if not np.isnan(std_y_error):
            self.y_deque.append(round(std_y_error,3))
        if not np.isnan(std_z_error):
            self.z_deque.append(round(std_z_error,3))

        self.counter =  self.counter + 1

        if self.counter > 10 and not self.paused:

            self.counter = 0

            self.axes_1.clear()
            n, bins, patches = self.axes_1.hist(list(self.x_deque), bins = 100, normed=True, facecolor='green', alpha=0.75, align='left', label='Histogram')
            (mu, sigma) = norm.fit(list(self.x_deque))
            y = mlab.normpdf( bins, mu, sigma)
            l = self.axes_1.plot(bins, y, 'r--', linewidth=2,label='Sampled Distribution')
            y = mlab.normpdf( bins, 0, 1)
            l = self.axes_1.plot(bins, y, 'b--', linewidth=2,label='Normal Distribution')
            self.axes_1.set_title(r'$\mathrm{Histogram\ of\ Camera\ Frame\ X\ Standard\ Error:}\ \mu=%.3f,\ \sigma=%.3f$' %(mu, sigma))
            self.axes_1.set_ylabel("Frequency")
            output= "Data Size: "+str(len(self.x_deque))
            self.axes_1.annotate(output, (0.05,0.9), xycoords = 'axes fraction')
            self.axes_1.set_xlim(-3 * sigma , 3 * sigma)
            self.axes_1.legend()

            self.axes_2.clear()
            n, bins, patches = self.axes_2.hist(list(self.y_deque), bins = 100, normed=True, facecolor='green', alpha=0.75, align='left', label='Histogram')
            (mu, sigma) = norm.fit(list(self.y_deque))
            y = mlab.normpdf( bins, mu, sigma)
            l = self.axes_2.plot(bins, y, 'r--', linewidth=2,label='Sampled Distribution')
            y = mlab.normpdf( bins, 0, 1)
            l = self.axes_2.plot(bins, y, 'b--', linewidth=2,label='Normal Distribution')
            self.axes_2.set_title(r'$\mathrm{Histogram\ of\ Camera\ Frame\  Y\ Standard\ Error:}\ \mu=%.3f,\ \sigma=%.3f$' %(mu, sigma))
            self.axes_2.set_ylabel("Frequency")
            output= "Data Size: "+str(len(self.y_deque))
            self.axes_2.annotate(output, (0.05,0.9), xycoords = 'axes fraction')
            self.axes_2.set_xlim(-3 * sigma , 3 * sigma)
            self.axes_2.legend()

            self.axes_3.clear()
            n, bins, patches = self.axes_3.hist(list(self.z_deque), bins = 100, normed=True, facecolor='green', alpha=0.75, align='left', label='Histogram')
            (mu, sigma) = norm.fit(list(self.z_deque))
            y = mlab.normpdf( bins, mu, sigma)
            l = self.axes_3.plot(bins, y, 'r--', linewidth=2,label='Sampled Distribution')
            y = mlab.normpdf( bins, 0, 1)
            l = self.axes_3.plot(bins, y, 'b--', linewidth=2,label='Normal Distribution')
            self.axes_3.set_title(r'$\mathrm{Histogram\ of\ Camera\ Frame\ Z\ Standard\ Error:}\ \mu=%.3f,\ \sigma=%.3f$' %(mu, sigma))
            self.axes_3.set_xlabel("Value")
            self.axes_3.set_ylabel("Frequency")
            output= "Data Size: "+str(len(self.z_deque))
            self.axes_3.annotate(output, (0.05,0.9), xycoords = 'axes fraction')
            self.axes_2.set_xlim(-3 * sigma , 3 * sigma)
            self.axes_3.legend()

            self.canvas.draw()

if __name__ == '__main__':
    rospy.init_node("online_histogram", anonymous=False)

    try:
        app = QApplication(sys.argv)
        verifyObj = verifyRGBD()
        verifyObj.show()
        app.exec_()
    except rospy.ROSInterruptException: pass
    rospy.spin()
