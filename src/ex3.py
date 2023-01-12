#!/usr/bin/env python3

import rospy
import waffle

node_name = "wall_follower"

rospy.init_node(node_name, anonymous=True)
rate = rospy.Rate(2) # hz
rospy.loginfo(f"{node_name}: Initialised.")

motion = waffle.Motion()
lidar = waffle.Lidar()

lin_vel = 0.0
ang_vel = 0.0
movement = ""

while not rospy.is_shutdown():

    lin_vel = 0.1

    wall_rate = lidar.distance.l3 - lidar.distance.l4
    print(lidar.distance)

    if (lidar.distance.front < 0.3) or (lidar.distance.l1 < 0.4):
        lin_vel = 0.0
        ang_vel = -0.3
        movement = "turning to avoid collision up ahead..."
    elif (lidar.distance.l3 > 0.6):
        movement = "lost sight of the wall, turning left..."
        lin_vel = 0.0
        ang_vel = 0.3
    elif abs(wall_rate) < 0.001:
        movement = "go straight"
        ang_vel = 0.0
    elif wall_rate < 0:
        movement = "turn right"
        ang_vel = -0.2 if lidar.distance.l3 > 0.2 else -0.4
    else:
        movement = "turn left"
        ang_vel = 0.2 if lidar.distance.l4 < 0.2 else 0.4
    
    print(f"{wall_rate=:.3f}")
    print(f"{movement=}\n")
    motion.move_at_velocity(linear=lin_vel, angular=ang_vel)
    rate.sleep()
    