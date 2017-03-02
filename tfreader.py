#!/usr/bin/env python
# coding:utf-8
import tf
import threading
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point

import numpy as np
from collections import Counter
from math import *
import time

import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt

class tfListener(threading.Thread):
    def __init__(self):  
        threading.Thread.__init__(self)
        self.listener = tf.TransformListener()
        print 'Waiting for tf ...'
        self.listener.waitForTransform('ORB_SLAM/World', 'ORB_SLAM/Robot', rospy.Time(), rospy.Duration(60.0))
        self.ox = 0
        self.oy = 0
        self.flag = True

    def run(self):
        while True:
            try:
                (trans,rot) = self.listener.lookupTransform('myRobot/odom', 'myRobot/base_link', rospy.Time())
                x = -trans[1]
                y = trans[0]
                plt.scatter(x, y, s=10, marker = 'o', label='real', c='r')

                (trans_c,rot_c) = self.listener.lookupTransform('ORB_SLAM/World', 'ORB_SLAM/Robot', rospy.Time())
                if self.flag == True:
                    x_c = trans_c[0]
                    y_c = trans_c[1]
                    self.ox = x - x_c
                    self.oy = y - y_c
                x_c = trans_c[0] + self.ox
                y_c = trans_c[1] + self.oy
                plt.scatter(x_c, y_c, s=10, marker = 'x', label='Robot center(ORB_SLAM)', c='b')
                    
                (trans_r,rot_r) = self.listener.lookupTransform('ORB_SLAM/World', 'ORB_SLAM/Camera', rospy.Time())
                x_r = trans_r[0] + self.ox
                y_r = trans_r[1] + self.oy
                plt.scatter(x_r, y_r, s=10, marker = '^', label='Camera center(ORB_SLAM)', c='g')
                if self.flag == True:
                    plt.legend()
                    self.flag = False
                plt.draw()
                time.sleep(0.3)

            except:
                return 

    def shutdown(self):
        rospy.loginfo("Stopping the robot (tfListener)...")

if __name__ == '__main__':
    try:       
        rospy.init_node('tfListener', anonymous=False)
        fig = plt.figure(figsize=(9, 9))
        axes = plt.gca()
        axes.set_xlim([-5, 5])
        axes.set_ylim([-3, 10])
        tf = tfListener()
        tf.start()
        plt.draw()
        plt.show()
    except KeyboardInterrupt:
        print 'interrupted!'

