#!/usr/bin/env python3

import rclpy
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args,
        signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('square')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    rate = node.create_rate(10)  # 10 Hz

    msg = Twist()
    msg.linear.x = 0.2  # Move forward at 0.2 m/s
    msg.angular.z = 0.0  # No rotation

    def stop():
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for i in range(5):
            publisher.publish(msg)
        node.destroy_node()
        rclpy.shutdown()
    
    try:
        while rclpy.ok():
            publisher.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        print("except...")
        stop()
    finally:
        
        print("finally...")

if __name__ == '__main__':
    main()
