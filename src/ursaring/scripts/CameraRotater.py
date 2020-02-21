#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import sys
import geometry_msgs.msg
import struct 
from time import sleep
import serial

if __name__ == '__main__':
    theta = 1 
	direction = 1 
	degree_step = 18
	step_ratio = 150.0/180
	time_per_scan = 1 #Seconds
    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = tf2_ros.TransformBroadcaster()
	ser = serial.Serial('/dev/ttyACM0', 9600) # Establish the connection on a specific port
	while not rospy.is_shutdown():
			transformStamped = geometry_msgs.msg.TransformStamped()

			transformStamped.header.stamp = rospy.Time.now()
			transformStamped.header.frame_id = "map"
			transformStamped.child_frame_id = "camera_link"

			transformStamped.transform.translation.x = float(0)
			transformStamped.transform.translation.y = float(0)
			transformStamped.transform.translation.z = float(0)
			# '23' != str(23)
			rospy.loginfo("Rotating " + str(int(step_ratio * degree_step * direction)) + " Steps to " + str(theta))

			

			if 0 < theta < 180:
				theta += int(degree_step * direction)
			else:
				direction = -1 if direction == 1 else 1 
				theta += int(degree_step * direction)
			ser.write('{}'.format(int(step_ratio * degree_step * direction)))
			sleep(time_per_scan)
			quat = tf.transformations.quaternion_from_euler(
				   float(0),float(theta),float(0))
			transformStamped.transform.rotation.x = quat[0]
			transformStamped.transform.rotation.y = quat[1]
			transformStamped.transform.rotation.z = quat[2]
			transformStamped.transform.rotation.w = quat[3]
			broadcaster.sendTransform(transformStamped)
			#sleep(.5)
			#rospy.spin()