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

from numpy import sin
import time

import sys
epsilon = sys.float_info.epsilon



class odomListener(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.pos_init = 0
        rospy.init_node('kelman', anonymous=False)
        rospy.Subscriber("/odom", Odometry, self.callback_Odom, queue_size=1)
        self.lastStamp = 0.0
        self.diffStamp = 0.0
        self.angular_velocity_x = 0
        self.v = 0
	self.psi = 0
        self.angular_velocity_z = 0

    def run(self):
        rospy.spin()

    def callback_Odom(self, data):
        curStamp = data.header.stamp.to_sec()
        self.diffStamp = curStamp - self.lastStamp
        self.lastStamp = curStamp
    	self.v = data.twist.twist.linear.x
	self.psi = data.twist.twist.angular.z


class kelman(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0]]).T
        self.dt = 0.1

    def run(self):
        """
        dt = 1.0/100.0
        P = np.diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0])
        print(P, P.shape)
        sGPS = 0.5 * 8.8 * dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
        sCourse = 0.1 * dt  # assume 0.1rad/s as maximum turn rate for the vehicle
        sVelocity = 8.8 * dt  # assume 0.12m/s2 as maximum acceleration, forcing the vehicle
        sYaw = 1.0 * dt  # assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle
        Q = np.diag([sGPS**2, sGPS**2, sCourse**2, sVelocity**2, sYaw**2])
        print(Q, Q.shape)

        varGPS = 6.0 # Standard Deviation of GPS Measurement
        varspeed = 1.0 # Variance of the speed measurement
        varyaw = 0.1 # Variance of the yawrate measurement
        R = np.matrix([[varGPS**2, 0.0, 0.0, 0.0],
                       [0.0, varGPS**2, 0.0, 0.0],
                       [0.0, 0.0, varspeed**2, 0.0],
                       [0.0, 0.0, 0.0, varyaw**2]])
        print(R, R.shape)
        """

        v = 0.10
        yaw = 0.10
        #x = np.array(self.x)
        x = self.x
        dt = self.dt
        x[3] = v
        x[4] = yaw

        if np.abs(x[4])<0.0001:
            x[0] = x[0] + x[3]*dt * np.cos(x[2])
            x[1] = x[1] + x[3]*dt * np.sin(x[2])
            x[2] = x[2]
            x[3] = x[3]
            x[4] = epsilon # avoid numerical issues in Jacobians
        else: # otherwise
            x[0] = x[0] + (x[3]/x[4]) * (np.sin(x[4]*dt+x[2]) - np.sin(x[2]))
            x[1] = x[1] + (x[3]/x[4]) * (-np.cos(x[4]*dt+x[2])+ np.cos(x[2]))
            x[2] = (x[2] + x[4]*dt + np.pi) % (2.0*np.pi) - np.pi
            x[3] = x[3]
            x[4] = x[4]
        
 
        self.x = x


if __name__ == '__main__':
    ekf = kelman()
    ekf.run()
    while True:
        ekf.run()
        print ekf.x
        time.sleep(1)

#imu = imuListener()
#imu.run()
