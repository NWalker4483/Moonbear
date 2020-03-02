#!/usr/bin/env python
import rospy
# because of transformations
import tf
import tf2_ros
import geometry_msgs.msg
from time import sleep
import time 

wheelBase=1.68
speed= (vR + vL)/ 2.0
v_th=speed*tan(wheelAngle)/ wheelBase
vx = speed
vy = 0.0
odomMsg.twist.twist.linear.x = vx
odomMsg.twist.twist.linear.y = vy
odomMsg.twist.twist.angular.z = v_th
odomMsg.header.frame_id = 'odom'
odomMsg.child_frame_id = 'base_footprint'