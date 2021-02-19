#!/usr/bin/env python3.6
import rospy
from geometry_msgs.msg import Twist

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import serial

def pid_callback(data):
  global ser
  ser.write(b'T{0}\nS{1}'.format(data, steering_angle))  


def cmd_callback(data):
  global pub
  global steering_angle
  steering_angle = int(90 + (data.angular.z * (180/3.14)))
  pub.publish(data.linear.x)

if __name__ == '__main__': 
  try:    
    steering_angle = 0
    rospy.init_node('Linear_PID_Controller')
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 

    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)

    pub = rospy.Publisher("/setpoint", Float64, queue_size=1)
    rospy.Subscriber("/control_effort", Float64, pid_callback, queue_size=1)

    ser = serial.Serial('/dev/ttyACM0')
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
