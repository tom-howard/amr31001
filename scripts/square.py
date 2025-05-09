#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from amr31001_modules import waffle
from math import sqrt, pow, pi 

node = waffle.create_node('square')
publisher = node.create_publisher(Twist, '/cmd_vel', 10)
rate = node.create_rate(10, node.get_clock())  # 10 Hz
velocity_msg = Twist()

def main():
    movement = "fwd" # "fwd" or "turn"
    transition = True
    timestamp = waffle.get_time(node)

    while rclpy.ok():
        elapsed_time = waffle.get_time(node) - timestamp 
        print(f"{elapsed_time}")
        if transition: 
            timestamp = waffle.get_time(node)
            transition = False
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            print(f"Transitioning into state: {movement}")
        elif movement == "fwd": 
            if elapsed_time > 2:
                movement = "turn"
                transition = True
            else:
                velocity_msg.linear.x = 0.05
                velocity_msg.angular.z = 0.0
        elif movement == "turn": 
            if elapsed_time > 4:
                movement = "fwd"
                transition = True
            else:
                velocity_msg.angular.z = 0.2
                velocity_msg.linear.x = 0.0
        publisher.publish(velocity_msg) 
        rate.sleep()

try:
    main()    
except KeyboardInterrupt:
    print("\nCtrl+C detected. Stopping the robot...")
    waffle.stop(node, publisher)
finally:
    print("Node stopped.")
