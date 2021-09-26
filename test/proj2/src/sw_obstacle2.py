#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int8

motor_control = xycar_motor()
steer_control = Int8()
lidar_points = None
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
pub_dir = rospy.Publisher('direction_msg', Int8, queue_size=1)
direction = None
count_right =0
count_left=0

def motor_pub(angle, speed): 
	global pub
	global motor_control

	motor_control.angle = angle
	motor_control.speed = speed

	pub.publish(motor_control)
 
def steer_pub(right_point, left_point):
  global pub_dir
  global count_right, count_left
  count_right = right_point
  count_left = left_point
  

  if (count_right >= 20):
    steer_control = 1

  elif (count_left >= 20):
    steer_control = 2
  else:
    steer_control = 3
    
  print(steer_control)
  pub_dir.publish(steer_control)
 

def lidar_callback(data):
  global lidar_points
  lidar_points = data.ranges
  
def dir_callback(data):
  global direction
  direction = data.data

rospy.init_node('my_lidar_sw', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
rospy.Subscriber("/direction_msg", Int8, dir_callback, queue_size=1)

while not rospy.is_shutdown():
  if lidar_points == None:
    continue
    
    rtn = ""
    #for i in range(12):
    #    rtn += str(format(lidar_points[i*30],'.2f')) + ", "
    for i in range(1, 505):
        rtn += str(format(lidar_points[i],'.2f')) + ", "

    print(rtn[:-2])
    print("------------------------------------------------------")
    
    

  temp1 =(int)(35*505)/360
  temp2 =(int)(75*505)/360
  temp3 =(int)(285*505)/360
  temp4 =(int)(325*505)/360



  count =0
  for i in range(temp1, temp2):

  
    if (lidar_points[i] < 0.8):
      count +=1
      #print("find: ",i)
      
      if (count >= 7):
        speed = -15
        angle = 50
        motor_pub(angle, speed) 
        time.sleep(0.5)
        print("right")
  count =0  
  
  for i in range(temp3, temp4):
    if (lidar_points[i] < 0.8):
      count +=1
      #print("find: ",i)
      
      if (count >= 7):
        speed = -15
        angle = 50
        motor_pub(angle, speed) 
        time.sleep(0.5)
        print("left")   
  
  speed = 15
  angle = 0
  motor_pub(angle, speed) 
  time.sleep(0.5)
  print("go")

        
  time.sleep(1.0)


