#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from reef_msgs.msg import DeltaToVel
import math
from collections import deque
import matplotlib.pyplot as plt
from scipy.stats import norm
import matplotlib.mlab as mlab

#!/usr/bin/env python
from PlotWindow import PlotWindow

import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *


class verifyRGBD(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)

        print("Verifying RGBD")
        truth_subs = Subscriber("velocity/body_level_frame", TwistWithCovarianceStamped)
        rgbd_subs = Subscriber("rgbd_velocity/body_level_frame", DeltaToVel)
        approx_sync_ = ApproximateTimeSynchronizer([ truth_subs, rgbd_subs], queue_size=5, slop=0.1)
        approx_sync_.registerCallback(self.callback)

        self.window_size = 10000
        self.counter = 0
        self.index=0
        self.x_deque= deque(maxlen=self.window_size)
        self.y_deque= deque(maxlen=self.window_size)
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

    def callback(self, mocap_vel, rgbd_msg):

        self.counter = self.counter + 1

        std_x = math.sqrt(rgbd_msg.vel.twist.covariance[0])
        std_y  = math.sqrt(rgbd_msg.vel.twist.covariance[7])

        std_error_x = (mocap_vel.twist.twist.linear.x - rgbd_msg.vel.twist.twist.linear.x)/std_x
        std_error_y =  (mocap_vel.twist.twist.linear.y - rgbd_msg.vel.twist.twist.linear.y)/std_y

        self.x_deque.append(round(std_error_x,3))
        self.y_deque.append(round(std_error_y,3))

        if self.counter > 10 and not self.paused:

            self.counter = 0

            self.axes_1.clear()
            n, bins, patches = self.axes_1.hist(list(self.x_deque), bins = 100, normed=True, facecolor='green', alpha=0.75, align='left', label='Histogram')
            (mu, sigma) = norm.fit(list(self.x_deque))
            y = mlab.normpdf( bins, mu, sigma)
            l = self.axes_1.plot(bins, y, 'r--', linewidth=2,label='Sampled Distribution')
            y = mlab.normpdf( bins, 0, 1)
            l = self.axes_1.plot(bins, y, 'b--', linewidth=2,label='Normal Distribution')
            self.axes_1.set_title(r'$\mathrm{Histogram\ of\ X\ Standard\ Error:}\ \mu=%.3f,\ \sigma=%.3f$' %(mu, sigma))
            self.axes_1.set_xlabel("Value")
            self.axes_1.set_ylabel("Frequency")
            output= "Data Size: "+str(len(self.x_deque))
            self.axes_1.annotate(output, (0.05,0.9), xycoords = 'axes fraction')
            self.axes_1.legend()



            self.axes_2.clear()
            n, bins, patches = self.axes_2.hist(list(self.y_deque), bins = 100, normed=True, facecolor='green', alpha=0.75, align='left', label='Histogram')
            (mu, sigma) = norm.fit(list(self.y_deque))
            y = mlab.normpdf( bins, mu, sigma)
            l = self.axes_2.plot(bins, y, 'r--', linewidth=2,label='Sampled Distribution')
            y = mlab.normpdf( bins, 0, 1)
            l = self.axes_2.plot(bins, y, 'b--', linewidth=2,label='Normal Distribution')
            self.axes_2.set_title(r'$\mathrm{Histogram\ of\ Y\ Standard\ Error:}\ \mu=%.3f,\ \sigma=%.3f$' %(mu, sigma))
            self.axes_2.set_xlabel("Value")
            self.axes_2.set_ylabel("Frequency")
            output= "Data Size: "+str(len(self.y_deque))
            self.axes_2.annotate(output, (0.05,0.9), xycoords = 'axes fraction')

            self.canvas.draw()

if __name__ == '__main__':
    rospy.init_node("verify_rgbd", anonymous=False)

    try:
        app = QApplication(sys.argv)
        verifyObj = verifyRGBD()
        verifyObj.show()
        app.exec_()
    except rospy.ROSInterruptException: pass
    rospy.spin()






