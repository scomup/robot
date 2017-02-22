import rospy
import serial
import string
import math
import threading
from time import time, sleep
from sensor_msgs.msg import Imu


def toEulerianAngle(x, y, z, w):
    ysqr = y * y
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.atan(t0/t1)

    t2 = +2.0 * (w * y - z * x)
    if t2 > 1.0:
        t2 = 1.0
    if t2 < -1.0:
        t2 = -1.0       
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z) 
    yaw = math.atan(t3/t4)
    return roll, pitch, yaw


class imuListener(threading.Thread):
    def __init__(self):  
        threading.Thread.__init__(self)
        self.pos_init = 0
        rospy.init_node('imuListener', anonymous=False)
        rospy.Subscriber("/myRobot/imu_data", Imu, self.callback_Imu, queue_size = 1)
        self.lastStamp = 0.0
        self.diffStamp = 0.0
        self.angular_velocity_x = 0
        self.angular_velocity_y = 0
        self.angular_velocity_z = 0

    def run(self):
        rospy.spin()
    def callback_Imu(self, data):
        curStamp = data.header.stamp.to_sec() 
        self.diffStamp = curStamp - self.lastStamp
        self.lastStamp = curStamp
        r, p, y = toEulerianAngle(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.angular_velocity_x = self.angular_velocity_x + data.linear_acceleration.x * self.diffStamp
        self.angular_velocity_y = self.angular_velocity_y + data.linear_acceleration.y * self.diffStamp
        self.angular_velocity_z = self.angular_velocity_z + data.linear_acceleration.z * self.diffStamp
        print self.angular_velocity_x,self.angular_velocity_y

    def shutdown(self):
        rospy.loginfo("Stopping the robot (scan)...")

imu = imuListener()
imu.run()