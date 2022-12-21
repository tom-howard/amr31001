#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Motion():
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def move_at_velocity(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
        self.publish()
    
    def stop(self):
        self.move_at_velocity()
        self.publish()

    def publish(self):
        self.publisher.publish(self.vel_cmd)

class Pose():
    def odom_cb(self, odom_data: Odometry):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        yaw = self.round(degrees(yaw), 4)
        self.yaw_direction = np.sign(yaw)
        self.yaw = abs(yaw)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.yaw_direction = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Lidar():
    
    class scanSubsets():
        def __init__(self):
            self.front = 0.0
            self.r1 = 0.0; self.r2 = 0.0; self.r3 = 0.0; self.r4 = 0.0
            self.l1 = 0.0; self.l2 = 0.0; self.l3 = 0.0; self.l4 = 0.0
        
        def __str__(self):
            return f"{self.l4:.3f}m, {self.l3:.3f}m, {self.l2:.3f}m, {self.l1:.3f}m, " \
                f"{self.front:.3f}m, " \
                f"{self.r1:.3f}m, {self.r2:.3f}m, {self.r3:.3f}m, {self.r4:.3f}m."
                        
    def laserscan_cb(self, scan_data: LaserScan):
        # front:
        left_arc = scan_data.ranges[0:20+1]
        right_arc = scan_data.ranges[-20:]
        full_arc = np.array(left_arc[::-1] + right_arc[::-1])
        valid = full_arc[full_arc>0.1]
        self.distance.front = valid.min() if np.shape(valid)[0] > 0 else np.nan
        
        # right subsets:
        range = np.array(scan_data.ranges[320:340+1])
        valid = range[range>0.1]
        self.distance.r1 = valid.min() if np.shape(valid)[0] > 0 else np.nan
        
        range = np.array(scan_data.ranges[300:320+1])
        valid = range[range>0.1]
        self.distance.r2 = valid.min() if np.shape(valid)[0] > 0 else np.nan
        
        range = np.array(scan_data.ranges[275:290+1])
        valid = range[range>0.1]
        self.distance.r3 = valid.min() if np.shape(valid)[0] > 0 else np.nan

        range = np.array(scan_data.ranges[255:270+1])
        valid = range[range>0.1]
        self.distance.r4 = valid.min() if np.shape(valid)[0] > 0 else np.nan

        # left subsets:
        range = np.array(scan_data.ranges[20:40+1])
        valid = range[range>0.1]
        self.distance.l1 = valid.min() if np.shape(valid)[0] > 0 else np.nan

        range = np.array(scan_data.ranges[40:60+1])
        valid = range[range>0.1]
        self.distance.l2 = valid.min() if np.shape(valid)[0] > 0 else np.nan

        range = np.array(scan_data.ranges[70:85+1])
        valid = range[range>0.1]
        self.distance.l3 = valid.min() if np.shape(valid)[0] > 0 else np.nan

        range = np.array(scan_data.ranges[95:110+1])
        valid = range[range>0.1]
        self.distance.l4 = valid.min() if np.shape(valid)[0] > 0 else np.nan
        
        self.wait_for_readings = False

    def __init__(self):
        self.distance = self.scanSubsets()
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
        self.wait_for_readings = True
        while self.wait_for_readings:
            continue
        print('LiDAR Data is available...')

