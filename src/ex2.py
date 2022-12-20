#!/usr/bin/env python3

import rospy
import waffle

node_name = "wall_detection"

rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(2) # hz
rospy.loginfo(f"{node_name}: Initialised.")

motion = waffle.Motion()
pose = waffle.Pose()
lidar = waffle.Lidar()

while not rospy.is_shutdown():
    lidar.distance.readings()
    if (lidar.distance.front < 0.4) or (lidar.distance.r1 < 0.3):
        motion.move_at_velocity(linear=0.0, angular=0.2)
    else:
        motion.move_at_velocity(linear=0.1, angular=0.0)
        
    rate.sleep()
    