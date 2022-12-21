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

lin_vel = 0.0
ang_vel = 0.0

while not rospy.is_shutdown():

    lin_vel = 0.05

    diff = lidar.distance.l3 - lidar.distance.l4
    print(diff)

    if abs(diff) < 0.001:
        print("straight")
        ang_vel = 0.0
    elif diff < 0:
        print("turn right")
        ang_vel = -0.2
        motion.move_at_velocity(angular=-0.2)
    else:
        print("turn left")
        ang_vel = 0.2
    motion.move_at_velocity(linear=lin_vel, angular=ang_vel)
    rate.sleep()
    