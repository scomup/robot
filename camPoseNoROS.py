#!/usr/bin/env python
# coding:utf-8
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from math import *
#from sys.float_info import epsilon
epsilon = sys.float_info.epsilon

from numpy import sin
import time

import sys
epsilon = sys.float_info.epsilon


class drawer():
    
    def __init__(self):
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
        fig.canvas.mpl_connect('key_press_event', self.press)


    def update(self):
        self.v = self.cmd[0]
        self.yawrate = self.cmd[1]
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
        
    def press(self, event):
        sys.stdout.flush()
        if event.key == 'a':
            self.cmd = (0,0.3)
        if event.key == 'd':
            self.cmd = (0,-0.3)
        if event.key == 'w':
            self.cmd = (0.4,0)
        if event.key == 's':
            self.cmd = (-0.4,0)
        if event.key == 'q':
            self.cmd = (0.2,0.3)
        if event.key == 'e':
            self.cmd = (0.2,-0.3)
        self.drawerOnce()

    def run(self):
        plt.show()

    def drawerOnce(self):
        self.update()
        i = np.matrix([[1],[0]])
        R = np.matrix([[cos(self.yaw),-sin(self.yaw)],
         [sin(self.yaw), cos(self.yaw)]])
        print self.yaw
        v =R*i
        plt.quiver(self.pose[0,0], self.pose[1,0], v[0,0], v[1,0],scale=30,color='r')
        campose = self.pose + R*self.campose
        plt.quiver(campose[0,0], campose[1,0], v[0,0], v[1,0],scale=30,color='g')
        plt.draw()
        if self.flag == True:
            plt.legend()
            self.flag = False


if __name__ == '__main__':
    fig = plt.figure(figsize=(9, 9))
    axes = plt.gca()
    axes.set_xlim([-5, 5])
    axes.set_ylim([-5, 5])
    drawer = drawer()
    drawer.run()
