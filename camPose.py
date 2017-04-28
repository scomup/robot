#!/usr/bin/env python
# coding:utf-8
import rospy
import serial
import string
from  math import *
import threading
import numpy as np
from time import time, sleep
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt

from numpy import sin
import time

import sys
epsilon = sys.float_info.epsilon


#TOPIC_ODOM = "/odom"
#TOPIC_CMD = "/cmd_vel"
TOPIC_ODOM = "/myRobot/odom"
TOPIC_CMD = "/myRobot/cmd_vel"


class cmdListener(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.v = 0.
        self.yawrate = 0.
        rospy.Subscriber(TOPIC_CMD, Twist, self.callback_Cmd)

    def run(self):
        rospy.spin()

    def callback_Cmd(self, msg):
        self.v = msg.linear.x
        self.yawrate = msg.angular.z


    def shutdown(self):
        rospy.loginfo("Stopping the robot cmdListener...")


class drawer(threading.Thread):
    
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.cmd = cmd
        self.flag = True
        self.pose = np.matrix([[0],[0]])
        self.yaw = 0.
        self.v = 0.
        self.yawrate = 0.
        self.dt = 0.3
        self.xc = 0.
        self.yc = 0.
        self.dpose = np.matrix([[0],[0]])
        self.campose = np.matrix([[.60],[.20]])


    def update(self):
        self.v = self.cmd.v
        self.yawrate = self.cmd.yawrate
        if np.abs(self.yawrate) < 0.0001:
            self.dpose = np.matrix([[self.v * self.dt * np.cos(self.yaw)],[self.v * self.dt * np.sin(self.yaw)]])
        else:  # otherwise
            self.dpose = np.matrix([[(self.v / self.yawrate) * (np.sin(self.yawrate * self.dt + self.yaw) - np.sin(self.yaw))],\
                                    [(self.v / self.yawrate) * (-np.cos(self.yawrate * self.dt + self.yaw) + np.cos(self.yaw))]])
            self.yaw = (self.yaw + self.yawrate * self.dt + np.pi) % (2.0 * np.pi) - np.pi
        self.pose = self.pose + self.dpose
        #print self.x, self.y
        #self.c = self.c+1
        #print self.c
        

    def run(self):
        try:
            plt.show(block=False)
            while True:
                self.update()
                i = np.matrix([[1],[0]])
                R = np.matrix([[cos(self.yaw),-sin(self.yaw)],
                 [sin(self.yaw), cos(self.yaw)]])
                print self.yaw

                v =R*i
                plt.quiver(self.pose[0,0], self.pose[1,0], v[0,0], v[1,0],scale=30,color='r')
                campose = self.pose + R*self.campose
                plt.quiver(campose[0,0], campose[1,0], v[0,0], v[1,0],scale=30,color='g')

                #plt.scatter(self.x, self.y, s=10, label='Kelman filter result', c='g')
                #print self.cmd.x, self.cmd.y
                plt.draw()
                if self.flag == True:
                    plt.legend()
                    self.flag = False
                plt.pause(self.dt)
        except KeyboardInterrupt:
            print 'interrupted!'






if __name__ == '__main__':
    rospy.init_node('kelman', anonymous=False)
    fig = plt.figure(figsize=(9, 9))
    axes = plt.gca()
    axes.set_xlim([-5, 5])
    axes.set_ylim([-5, 5])
    cmd = cmdListener()
    cmd.start()
    drawer = drawer(cmd)
    drawer.start()
    plt.show()
