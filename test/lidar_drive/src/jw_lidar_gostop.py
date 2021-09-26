#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
lidar_points = None

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges

rospy.init_node('my_lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
#def drive_stop():
  
#def drive_go():

while not rospy.is_shutdown():
  if lidar_points == None:
    continue
    
  rtn = ""
  #for i in range(12):
  #    rtn += str(format(lidar_points[i*30],'.2f')) + ", "
  for i in range(1, 505):
    rtn += str(format(lidar_points[i],'.2f')) + ", "

  #print(rtn[:-2])
  #print("------------------------------------------------------")
    
        
  center1 = (15 * 505) / 360
  center2 = (345 * 505 ) / 360
  
  #if 
  rtn = rtn.split(',')
  #print(center1)
  #print(center2)
  #print(rtn[0:center1])
  #print()
  range1 = rtn[0:center1]
  #print(rtn[center2:-2])
  #print(rtn[center2:505])
  #print(len(rtn[center2:505]))
  range2 =rtn[center2:505]
  #range1 = range1 + range2
  #range1.apeend(range2)
  
  #print(len(range1))
  #print(type(range1))
  count =0
  #for i in range1:
  #  print(i)
  #print(type(range1[0]))
  
  for i in range1:
    #speed = 10
    #pub.publish(angle,speed)
    #time.sleep(0.5)
    if float(i) < 2.0 and float(i) != 0.0:
      count += 1
    if count > 4:
      #speed = 0
      print("stop")
      break
    else:
      print("go")
      #pub.publish(angle,speed)
      #time.sleep(0.5)
    
  print("------------------------------------------------------")
  time.sleep(1.0)
