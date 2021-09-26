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

# angle = 0
while pub.get_num_connections() == 0:
  speed = 0
  angle = 0
  for i in range(5):
    motor_pub(angle, speed) 
    time.sleep(0.5)
    print("stop")
  
speed = 15
count =0
while not rospy.is_shutdown():
  angle = 0
  for i in range(5):
    motor_pub(angle, speed) 
    time.sleep(0.5)
    print("check2")
  
  angle = 50
  for i in range(13):
    motor_pub(angle, speed)
    time.sleep(0.5)
    count +=1
    print("howmany: ",count)


  angle = 0
  for i in range(5):
    motor_pub(angle, speed) 
    time.sleep(0.5)
    print("check2")
    
  angle = -50
  for i in range(13):
    motor_pub(angle, speed) 
    time.sleep(0.5)
    print("check3")
    	

