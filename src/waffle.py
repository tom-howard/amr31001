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
        if abs(linear) > 0.26:
            lin_org = linear
            linear = np.sign(linear) * 0.26
            print(f"LINEAR velocity limited to {linear} m/s ({lin_org} m/s was requested).")

        if abs(angular) > 1.82:
            ang_org = angular
            angular = np.sign(angular) * 1.82
            print(f"ANGULAR velocity limited to {angular} rad/s ({ang_org} rad/s was requested).")
        
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

        self.wait_for_odom = False
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.yaw_direction = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.timestamp = rospy.get_time()
        self.wait_for_odom = True
        print('Waiting for Odometry...')
        while self.wait_for_odom:
            continue
        print('Odometry is ready.')
    
    def print(self):
        if rospy.get_time() - self.timestamp > 1:
            self.timestamp = rospy.get_time()
            print(f"posx = {self.posx:.3f} (m) posy = {self.posy:.3f} (m), yaw = {self.yaw:.1f} (degrees)")

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
            msg = f"              l1     front     r1          \n" \
                  f"       l2     {self.l1:<5.3f}  {self.front:^5.3f}  {self.r1:>5.3f}     r2       \n" \
                  f"l3     {self.l2:<5.3f}                       {self.r2:>5.3f}   r3\n" \
                  f"{self.l3:<5.3f}                                   {self.r3:>5.3f}\n" \
                  f"{self.l4:<5.3f} <-- l4                     r4 --> {self.r4:>5.3f}"
            return msg
                        
    def laserscan_cb(self, scan_data: LaserScan):

        def min_of_subset(start_index, stop_index):
            range = np.array(scan_data.ranges[start_index: stop_index+1])
            valid_data = range[range>0.1]
            return valid_data.mean() if np.shape(valid_data)[0] > 0 else np.nan

        # front:
        left_arc = scan_data.ranges[0:20+1]
        right_arc = scan_data.ranges[-20:]
        full_arc = np.array(left_arc[::-1] + right_arc[::-1])
        valid = full_arc[full_arc>0.1]
        self.distance.front = valid.mean() if np.shape(valid)[0] > 0 else np.nan
        
        # right subsets:
        self.distance.r1 = min_of_subset(320, 340)
        self.distance.r2 = min_of_subset(300, 320)
        self.distance.r3 = min_of_subset(275, 290)
        self.distance.r4 = min_of_subset(250, 265)
        
        # left subsets:
        self.distance.l1 = min_of_subset(20, 40)
        self.distance.l2 = min_of_subset(40, 60)
        self.distance.l3 = min_of_subset(70, 85)
        self.distance.l4 = min_of_subset(95, 110)
        
        self.wait_for_lidar = False

    def __init__(self):
        self.distance = self.scanSubsets()
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
        self.wait_for_lidar = True
        print('Waiting for LiDAR...')
        while self.wait_for_lidar:
            continue
        print('LiDAR is ready.')

