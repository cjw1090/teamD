#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
#from xycar_msgs.msg import xycar_motor
from ar_dqn_end import dqn_lidar_main
#from yolo_steering import *
#from sensor_msgs.msg import LaserScan



ultra = [0,0,0,0,0]

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
ar_dqnEnd_flag = False
temp = 0

  
def drive(angle, speed):
  # 모터제어 토픽을 발행하는 코드
  global motor_pub, xycar_msg 
  #motor_msg.header.stamp = rospy.Time.now()
  xycar_msg.angle = angle
  xycar_msg.speed = speed
  #print(motor_msg)
  motor_pub.publish(xycar_msg)
  
def callback(msg):
    global arData
    global ar_dqnEnd_flag
    global temp
    for i in msg.markers:
        if (i.id == 4) :
          #print("check!!!!!!!!!!!!")
          ar_dqnEnd_flag = True
          arData["DX"] = i.pose.pose.position.x
          arData["DY"] = i.pose.pose.position.y
          arData["DZ"] = i.pose.pose.position.z
  
          arData["AX"] = i.pose.pose.orientation.x
          arData["AY"] = i.pose.pose.orientation.y
          arData["AZ"] = i.pose.pose.orientation.z
          arData["AW"] = i.pose.pose.orientation.w
          temp = i.id
        if i.id == 5 or i.id == 6:
          #drive(0,0)
          ar_dqnEnd_flag = False
        #print("hihihihihih")
        




rospy.init_node('ar_go')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
#rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)

#motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )
#rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
#xycar_msg = xycar_motor()
    

  
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    #global xycar_msg
    
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"], arData["AY"],
                                            arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    print("=======================")
    print(" roll  : " + str(round(roll,1)))
    print(" pitch : " + str(round(pitch,1)))
    print(" yaw   : " + str(round(yaw,1)))

    print(" x : " + str(round(arData["DX"],4)))
    print(" y : " + str(round(arData["DY"],4)))
    print(" z : " + str(round(arData["DZ"],4)))
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    print("distance " + str(distance))
   # print("why~~~~~")
    rate = rospy.Rate(10)
    
    if (ar_dqnEnd_flag):
      dqn_lidar_main()
      print("lidar")
      #drive(angle, speed)
    """
    if(ar_dqnEnd_flag):
      #print("ar_s")
      if distance > 0.53 :
          #if pitch < -:
              angle = pitch 
          #  drive(, 10)
          #else:
          #  drive(0,15)
          if pitch > -45 :
              drive(-15, 10)
          else:
              drive(0,15)
      else:
          
          for _ in range(17):
            drive(-25, 15)
            rate.sleep()
          break
    

      if arData["DX"] < 0.39 :
          drive(-10,10)
          
      if arData["DZ"] > 0.365 :
          drive(0,15)
      else:
          #for _ in range(10):
          #  drive(0,10)
          #for _ in range(20):
          #  drive(-25, 10)
          for _ in range(20):
            drive(-25, 10)
          break
            #rate.sleep()

    print("=======================")
    print(" roll  : " + str(round(roll,1)))
    print(" pitch : " + str(round(pitch,1)))
    print(" yaw   : " + str(round(yaw,1)))

    print(" x : " + str(round(arData["DX"],4)))
    print(" y : " + str(round(arData["DY"],4)))
    print(" z : " + str(round(arData["DZ"],4)))
    print("angle : " + str(round(xycar_msg.angle,0)))
    print("speed : " + str(round(xycar_msg.speed,0)))
    print("id : " + str(temp))
    """    
    
    #print("ultra : " + str(ultra))
    
    #img = np.zeros((100,500,3))
    
    #img = cv2.line(img,(25,65), (475,65),(0,0,255),2)
    #img = cv2.line(img,(25,40), (25,90),(0,0,255),3)
    #img = cv2.line(img,(250,40), (250,90),(0,0,255),3)
    #img = cv2.line(img,(475,40), (475,90),(0,0,255),3)
    
    #point = int(arData["DX"]) + 250
    
    #if point > 475 :
    #    point = 475
    #elif point < 25 :
    #    point = 25
        
    #img = cv2.circle(img,(point, 65), 15, (0,255,0), -1)
    
    #distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    #cv2.putText(img, str(int(distance)), (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
    
    #dx_dy_yaw = "DX:"+str(int(arData["DX"]))+ "DY:"+str(int(arData["DY"])) + "Yaw:"+str(round(yaw,1))
    #cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    
    #cv2.imshow('AR Tag Position', img)
    #cv2.waitKey(1)  
    #print(dx_dy_yaw)
    


	

#cv2.destrouAllWindows()