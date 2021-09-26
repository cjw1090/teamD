#!/usr/bin/env python

import rospy
import time
#from xycar_motor.msg import xycar_motor
from xycar_msgs.msg import xycar_motor

motor_control = xycar_motor()

rospy.init_node('auto_driver')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def motor_pub(angle, speed): 
	global pub
	global motor_control

	motor_control.angle = angle
	motor_control.speed = speed

	pub.publish(motor_control)


angle = 10
while not rospy.is_shutdown():
        """	
speed = 0
	for i in range(5):
		motor_pub(angle, speed)
		time.sleep(0.5)
	speed = 20
	for i in range(10):
		motor_pub(angle, speed) 
		time.sleep(0.5)
	speed = 0
	for i in range(5):
		motor_pub(angle, speed) 
		time.sleep(0.5)
        """


  speed = 20
	for i in range(10):
  	motor_pub(angle, speed) 
  	time.sleep(0.5)
