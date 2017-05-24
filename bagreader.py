#!/usr/bin/python
# coding: UTF-8
import numpy as np
import rosbag
from std_msgs.msg import Int32, String
import rospy, math, random
import numpy as np
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import tf
from matplotlib.widgets import Button
from geometry_msgs.msg import *


class BagReader:
    def __init__(self, bagfile, scan_topic, odom_topic):
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.points = []
        self.odoms = []
        self.data = []
        print "Bag file reading..."
        self.bag = rosbag.Bag(bagfile, 'r')
        print "Scan data reading..."
        self.readscan()
        print "Odom data reading..."
        self.readodom()
        print "Sync..."
        self.sync()
        self.bag.close()

    def readscan(self):
        laser_projector = LaserProjection()
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.scan_topic]):
            cloud = laser_projector.projectLaser(msg)
            frame_points = np.zeros([0,2])
            for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
                    p2d = np.array([p[0], p[1]])
                    frame_points = np.vstack([frame_points, p2d])
            self.points.append([time_stamp,frame_points])

    def readodom(self):
        laser_projector = LaserProjection()
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.odom_topic]):
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            t = tf.transformations.quaternion_matrix((qx,qy,qz,qw))
            t[0,3] = msg.pose.pose.position.x
            t[1,3] = msg.pose.pose.position.y
            t[2,3] = msg.pose.pose.position.z
            self.odoms.append([time_stamp,t])

    def sync(self):
        idx = 0
        start_time =self.points[0][0] + rospy.Duration(80, 0)
        for time_stamp_scan,scan_data in self.points:
            if time_stamp_scan < start_time:
                continue
            time_stamp_odom,odom_data = self.odoms[idx]
            while idx < len(self.odoms):
                if time_stamp_odom > time_stamp_scan:
                    break
                time_stamp_odom,odom_data = self.odoms[idx]
                idx+=1
            self.data.append((scan_data,odom_data))
            


class GUI(object):
    def __init__(self, bagreader):
        self.idx = 0
        self.data = bagreader.data
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        axprev = plt.axes([0.7, 0.05, 0.1, 0.075])
        axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
        tmp = tf.transformations.euler_matrix(0.0, 0.0, 3.)
        tmp[0,3] = 0.15
        self.scan_base = np.matrix(tmp)
        self.init_pose = np.matrix([[0],[0],[0],[1]])

        bnext = Button(axnext, 'Next')
        bnext.on_clicked(self.next)
        bprev = Button(axprev, 'Previous')
        bprev.on_clicked(self.prev)
        self.i = 0
        plt.show()

    def next(self, event):
        try:
            self.scan_show.remove()
            self.odom_show.remove()
        except:
            pass
            """
        self.i += 0.2
        scan = self.data[self.idx][0]
        odom = np.matrix(self.data[self.idx][1])
        scan_size = scan.shape[0]
        scan = scan.transpose()
        tmp = np.zeros((1, scan_size))
        scan = np.vstack([scan, tmp])
        tmp.fill(1)
        scan = np.vstack([scan, tmp])
        scan = np.matrix(scan)
        tmp = tf.transformations.euler_matrix(0.0, 0.0, self.i)
        R = np.matrix(tmp)
        scan = odom*R*scan
        print self.i%3.1415926
        scan = odom*scan
        curp = odom*self.init_pose 
        v = np.matrix([[1],[0]])
        v= odom[0:2,0:2]*v
        self.odom_show = self.ax.quiver(curp[0,0], curp[1,0], v[0,0], v[1,0], units='width')
        self.scan_show = self.ax.scatter(scan[0,:],scan[1,:])
        plt.draw()
"""
        self.idx += 1
        scan = self.data[self.idx][0]
        odom = np.matrix(self.data[self.idx][1])
        scan_size = scan.shape[0]
        scan = scan.transpose()
        tmp = np.zeros((1, scan_size))
        scan = np.vstack([scan, tmp])
        tmp.fill(1)
        scan = np.vstack([scan, tmp])
        scan = np.matrix(scan)
        scan = odom*self.scan_base*scan
        scan = odom*scan
        curp = odom*self.init_pose 
        v = np.matrix([[1],[0]])
        v= odom[0:2,0:2]*v
        self.odom_show = self.ax.quiver(curp[0,0], curp[1,0], v[0,0], v[1,0], units='width')
        self.scan_show = self.ax.scatter(scan[0,:],scan[1,:])
        plt.draw()

    def prev(self, event):
        try:
            self.scan_show.remove()
            self.odom_show.remove()
        except:
            pass
        self.idx -= 1
        scan = self.data[self.idx][0]
        odom = np.matrix(self.data[self.idx][1])
        scan_size = scan.shape[0]
        scan = scan.transpose()
        tmp = np.zeros((1, scan_size))
        scan = np.vstack([scan, tmp])
        tmp.fill(1)
        scan = np.vstack([scan, tmp])
        scan = np.matrix(scan)
        scan = odom*self.scan_base*scan
        curp = odom*self.init_pose 
        v = np.matrix([[1],[0]])
        v= odom[0:2,0:2]*v
        self.odom_show = self.ax.quiver(curp[0,0], curp[1,0], v[0,0], v[1,0], units='width')
        self.scan_show = self.ax.scatter(scan[0,:],scan[1,:])
        plt.draw()



bagreader = BagReader('h1.bag', '/Rulo/laser_scan', '/Rulo/odom')
gui = GUI(bagreader)