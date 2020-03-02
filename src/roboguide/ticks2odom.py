#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDriveStamped
from numpy import sin, cos
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)

def tck_callback(data):
	global wheelbase
	global steering_angle
	global last_msg
	global pub
	delta_time = rospy.Time.now() - last_msg # seconds
	speed = a * motor_value + b
steering_angle = a * motor_value + b
	angular_velocity = speed * tan(steering_angle) / wheelbase_ # rad or degree 
	ros::Duration dt = time_duration

	double x_dot = current_speed * cos(yaw_)
	double y_dot = current_speed * sin(yaw_)
	x_ += x_dot * dt.toSec()
	y_ += y_dot * dt.toSec()

	yaw_ += current_angular_velocity * dt.toSec()


	odom.pose.pose.position.x = x_
	odom.pose.pose.position.y = y_
	odom.pose.pose.orientation.x = 0.0
	odom.pose.pose.orientation.y = 0.0
	odom.pose.pose.orientation.z = sin(yaw_/2.0)
	odom.pose.pose.orientation.w = cos(yaw_/2.0)

	# Co variance should be cacluated empirically
	odom.pose.covariance[0]  = 0.2 ///< x
	odom.pose.covariance[7]  = 0.2 ///< y
	odom.pose.covariance[35] = 0.4 ///< yaw

	# Velocity
	odom.twist.twist.linear.x = current_speed
	odom.twist.twist.linear.y = 0.0
	odom.twist.twist.angular.z = current_angular_velocity	
	
	odomMsg.header.frame_id = 'odom'
	odomMsg.child_frame_id = 'base_footprint'

	wheelBase=1.68
	speed= (vR + vL) / 2.0 # m/s 
	v_th=speed*tan(wheelAngle)/ wheelBase
	vx = speed
	vy = 0.0
	odomMsg.twist.twist.linear.x = vx
	odomMsg.twist.twist.linear.y = vy
	odomMsg.twist.twist.angular.z = v_th
	odomMsg.header.frame_id = 'odom'
	odomMsg.child_frame_id = 'base_footprint'
	pub.publish(odomMsg)
	last_msg = 0

def cmd_callback(data):
  global wheelbase
  global steering_angle
  global last_msg
  global pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  pub.publish(msg)
  
if __name__ == '__main__': 
  try:    
    
	rospy.init_node('cmd_vel_to_ackermann_drive')

    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    wheelbase = rospy.get_param('~wheelbase', 10.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber("/ackermann_cmd", Twist, cmd_callback, queue_size=1)
    rospy.Subscriber("/ticks", Int16, tck_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
