#!/usr/bin/env python
# coding:utf-8
import rospy
import serial
import string
import math
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


class odomListener(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        rospy.Subscriber(TOPIC_ODOM, Odometry,
                         self.callback_Odom, queue_size=1)
        self.x = 0
        self.y = 0

    def run(self):
        rospy.spin()

    def callback_Odom(self, data):
        self.x = data.pose.pose.position.x  + np.random.normal(0, 0.3)
        self.y = data.pose.pose.position.y  + np.random.normal(0, 0.3)
        self.rx = data.pose.pose.position.x
        self.ry = data.pose.pose.position.y

    def shutdown(self):
        rospy.loginfo("Stopping the robot odomListener...")


class cmdListener(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.v = 0
        self.yawrate = 0
        rospy.Subscriber(TOPIC_CMD, Twist, self.callback_Cmd)

    def run(self):
        rospy.spin()

    def callback_Cmd(self, msg):
        self.v = msg.linear.x
        self.yawrate = msg.angular.z

    def shutdown(self):
        rospy.loginfo("Stopping the robot cmdListener...")


class kelman(threading.Thread):

    def __init__(self, cmd, measurment):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0]]).T
        self.px = self.x
        self.dt = 0.1
        self.cmd = cmd
        self.measurment = measurment
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])
        # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
        sGPS = 0.5 * 8.8 * self.dt**2
        sCourse = 0.1 * self.dt  # assume 0.1rad/s as maximum turn rate for the vehicle
        # assume 0.12m/s2 as maximum acceleration, forcing the vehicle
        sVelocity = 8.8 * self.dt
        # assume 1.0rad/s2 as the maximum turn rate acceleration for the
        # vehicle
        sYaw = 1.0 * self.dt
        self.Q = np.diag([sGPS**2, sGPS**2, sCourse**2, sVelocity**2, sYaw**2])
        self.H = np.matrix(
            [[1.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 1.0, 0.0, 0.0, 0.0]])
        varGPS = 100.0  # Standard Deviation of GPS Measurement
        varspeed = 1.0  # Variance of the speed measurement
        varyaw = 0.1  # Variance of the yawrate measurement
        self.R = np.matrix([[varGPS**2, 0.0],
                            [0.0, varGPS**2]])
        self.I = np.eye(5)

    def run(self):
        try:
            while True:
                self.update()
                time.sleep(self.dt)
        except KeyboardInterrupt:
            print 'interrupted!'

    def fun(self, x, u):
        x[3] = u[0]
        x[4] = u[1]
        if np.abs(x[4]) < 0.0001:
            x[0] = x[0] + x[3] * self.dt * np.cos(x[2])
            x[1] = x[1] + x[3] * self.dt * np.sin(x[2])
            x[2] = x[2]
            x[3] = x[3]
            x[4] = 0.000000001  # avoid numerical issues in Jacobians
        else:  # otherwise
            x[0] = x[0] + (x[3] / x[4]) * \
                (np.sin(x[4] * self.dt + x[2]) - np.sin(x[2]))
            x[1] = x[1] + (x[3] / x[4]) * \
                (-np.cos(x[4] * self.dt + x[2]) + np.cos(x[2]))
            x[2] = (x[2] + x[4] * self.dt + np.pi) % (2.0 * np.pi) - np.pi
            x[3] = x[3]
            x[4] = x[4]
        return x

    def Jacobians(self, x):
        a13 = float((x[3] / x[4]) *
                    (np.cos(x[4] * self.dt + x[2]) - np.cos(x[2])))
        a14 = float((1.0 / x[4]) *
                    (np.sin(x[4] * self.dt + x[2]) - np.sin(x[2])))
        a15 = float((self.dt * x[3] / x[4]) * np.cos(x[4] * self.dt + x[2]) - (
            x[3] / x[4]**2) * (np.sin(x[4] * self.dt + x[2]) - np.sin(x[2])))
        a23 = float((x[3] / x[4]) *
                    (np.sin(x[4] * self.dt + x[2]) - np.sin(x[2])))
        a24 = float((1.0 / x[4]) *
                    (-np.cos(x[4] * self.dt + x[2]) + np.cos(x[2])))
        a25 = float((self.dt * x[3] / x[4]) * np.sin(x[4] * self.dt + x[2]) - (
            x[3] / x[4]**2) * (-np.cos(x[4] * self.dt + x[2]) + np.cos(x[2])))
        J = np.matrix([[1.0, 0.0, a13, a14, a15],
                       [0.0, 1.0, a23, a24, a25],
                       [0.0, 0.0, 1.0, 0.0, self.dt],
                       [0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 1.0]])
        return J

    def update(self):
        if cmd.v > 0.3:
            pass
        #Prediction
        # ========================
        u = [cmd.v, cmd.yawrate]
        self.x = self.fun(self.x, u)
        self.px = self.fun(self.px, u)
        J = self.Jacobians(self.x)
        self.P = J * self.P * J.T + self.Q
        # ========================

        #Correction
        # ========================
        S = self.H * self.P * self.H.T + self.R
        K = (self.P * self.H.T) * np.linalg.inv(S)
        Z = np.matrix([[self.measurment.x, self.measurment.y]]).T
        y = Z - (self.H * self.x)
        self.x = self.x + (K * y)
        self.P = (self.I - (K * self.H)) * self.P
        print 'K:%5.4f' % (K[0,0])
        # ========================


class drawer(threading.Thread):
    
    def __init__(self, ekf, pose):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.ekf = ekf
        self.pose = pose
        self.flag = True

    def run(self):
        try:
            plt.show(block=False)
            while True:
                plt.scatter(self.ekf.x[0], self.ekf.x[1], s=10, label='Kelman filter result', c='g')
                plt.scatter(self.ekf.px[0], self.ekf.px[1], s=10, marker = 'x', label='Pose predicted by odom', c='b')
                plt.scatter(self.pose.x, self.pose.y, s=10, marker = 'o', label='Measurement with noise', c='c')
                plt.scatter(self.pose.rx, self.pose.ry, s=10, marker = 'o', label='real', c='r')
                plt.draw()
                if self.flag == True:
                    plt.legend()
                    self.flag = False
                plt.pause(0.1)
        except KeyboardInterrupt:
            print 'interrupted!'






if __name__ == '__main__':
    rospy.init_node('kelman', anonymous=False)
    fig = plt.figure(figsize=(9, 9))
    axes = plt.gca()
    axes.set_xlim([-3, 10])
    axes.set_ylim([-3, 10])
    cmd = cmdListener()
    pose = odomListener()
    cmd.start()
    ekf = kelman(cmd, pose)
    ekf.start()
    drawer = drawer(ekf, pose)
    drawer.start()
    plt.show()
