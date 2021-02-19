#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from std_msgs.msg import Float64
#import serial


def odom_callback(data):
  global state_pub
  rospy.loginfo(data)
  state_pub.publish(data.twist.twist.linear.x)

def pid_callback(data):
  global ser
  #ser.write(b'T{0}\nS{1}\n'.format(data, steering_angle))  


def cmd_callback(data):
  global set_pub
  global steering_angle
  steering_angle = int(90 + (data.angular.z * (180/3.14)))
  set_pub.publish(data.linear.x)

if __name__ == '__main__': 
  try:
    steering_angle = 0
    rospy.init_node('Linear_PID_Controller')
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    odom_topic = rospy.get_param('~odom_topic', "/camera/odom/sample") 

    set_pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
    rospy.Subscriber("/control_effort", Float64, pid_callback, queue_size=1)
    state_pub = rospy.Publisher("/state", Float64, queue_size=1)

    rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=1)
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)

    #ser = serial.Serial('/dev/ttyACM0')
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
