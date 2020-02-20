#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

control_pins = [31,33,35,37]
'''
halfstep_seq = [
  [1,0,0,0],
  [1,1,0,0],
  [0,1,0,0],
  [0,1,1,0],
  [0,0,1,0],
  [0,0,1,1],
  [0,0,0,1],
  [1,0,0,1]
]
'''

def callback(msg):
    	rospy.loginfo()
	'''
	for i in range(512):
		for halfstep in range(8):
			for pin in range(4):
				GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
			time.sleep(0.001)
	'''
if __name__ == '__main__':
	GPIO.setmode(GPIO.BOARD)
	control_pins = [31,33,35,37]
	for pin in control_pins:
	  GPIO.setup(pin, GPIO.OUT)
	  GPIO.output(pin, 0)
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off.
	rospy.init_node('camera_platform')
	# TODO Change to a /camera_angle topic 
	rospy.Subscriber("cmd_vel", Twist, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	GPIO.cleanup()
