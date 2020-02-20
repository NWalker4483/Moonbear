#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
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

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

for i in range(512):
  for halfstep in range(8):
    for pin in range(4):
      GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
    time.sleep(0.001)

if __name__ == '__main__':
	GPIO.setmode(GPIO.BOARD)
	control_pins = [7,11,13,15]
	for pin in control_pins:
	  GPIO.setup(pin, GPIO.OUT)
	  GPIO.output(pin, 0)
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off.
	rospy.init_node('camera_platform')

	rospy.Subscriber("chatter", String, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	GPIO.cleanup()
