#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

motor_control = xycar_motor()
lidar_points = None
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def motor_pub(angle, speed): 
	global pub
	global motor_control

	motor_control.angle = angle
	motor_control.speed = speed

	pub.publish(motor_control)

def lidar_callback(data):
  global lidar_points
  lidar_points = data.ranges

rospy.init_node('my_lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

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
    
    
  count =0
  temp1 =(int)(35*505)/360
  temp2 =(int)(75*505)/360
  temp3 =(int)(285*505)/360
  temp4 =(int)(325*505)/360
  check = "go"
  
  for i in range(temp1, temp2):
    if (lidar_points[i] < 0.7):
      count +=1
      #print("find: ",i)
      
      
      if (count >= 20):
        check = "right"
        speed = 15
        angle = 50
        motor_pub(angle, speed) 
        time.sleep(0.5) 
        print("right") 
        break
        
  count =0  
  
  
  for i in range(temp3,temp4):
    if (lidar_points[i] < 0.7):
      count +=1
      #print("find: ",i)
      
      if (count >= 20):
        speed = 15
        angle = -50
        motor_pub(angle, speed) 
        time.sleep(0.5) 
        print("left") 
        check = "left"
        break
        
#  if check == "right" :
#      speed = 15
#      angle = 50
#      motor_pub(angle, speed) 
#      time.sleep(0.5) 
#      print("right")       
#  elif check == "left" :
#      speed = 15
#      angle = -50
#      motor_pub(angle, speed) 
#      time.sleep(0.5)
#      print("left")
#  else:
#      speed = 15
#      angle = 0
#      motor_pub(angle, speed) 
#      time.sleep(0.5)
#      print("go")

  speed = 15
  angle = 0
  motor_pub(angle, speed) 
  time.sleep(0.5)
#      print("go")     
  time.sleep(1.0)

