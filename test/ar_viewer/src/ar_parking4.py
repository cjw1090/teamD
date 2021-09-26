#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
ar_parking_flag = True
ar_drive_flag = False
temp = 0
angle = 0
speed = 0
final_drive_flag = True
atan =0



def callback(msg):
    global arData
    global ar_parking_flag, ar_drive_flag, temp, atan
    
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        
        
        temp = i.id
        if(temp == 0):
          ar_parking_flag = False
          #print("parking start")
        
        if (temp == 9) :
          ar_drive_flag = True
          #print("check")
    atan = math.degrees(math.atan2(arData["DX"], arData["DY"]))

rospy.init_node('ar_parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )
xycar_msg = xycar_motor()


def back_drive1(ang,cnt):
    global xycar_msg, motor_pub
    global arData
    global angle, speed
    print("back1--------------------------")
    for cnt in range(cnt):
        xycar_msg.angle = -ang
        #xycar_msg.angle = -2.5*(-atan+pitch)
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        if arData["DX"] < -0.03:
          break
        time.sleep(0.1)
    for cnt in range(cnt):
        xycar_msg.angle = ang
        #xycar_msg.angle = 2.5*(-atan+pitch)
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        if abs(pitch) < 3:
          break
        time.sleep(0.1)
        
def back_drive2(ang,cnt):
    global xycar_msg, motor_pub
    global arData
    global angle, speed
    print("back2--------------------------")
    for cnt in range(cnt):
        xycar_msg.angle = ang
        #xycar_msg.angle = 2.5*(-atan+pitch)
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        if arData["DX"] < 0.03:
          break
        time.sleep(0.1)
    for cnt in range(cnt):
        xycar_msg.angle = -ang
        #xycar_msg.angle = -2.5*(-atan+pitch)
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        if abs(pitch) < 3:
          break
        time.sleep(0.1)

def left_drive(ang,cnt):
    global xycar_msg, motor_pub
    global ar_drive_flag
    global angle, speed
    global arData
    #print("go left--------------------------")
    
    while (arData["DZ"] > 0.5):
        if(arData["DX"] < 0):
          xycar_msg.angle = -20
          xycar_msg.speed = 10
          motor_pub.publish(xycar_msg)
        else:
          xycar_msg.angle = 20
          xycar_msg.speed = 10
          motor_pub.publish(xycar_msg)
    
    for i in range(30):
          xycar_msg.angle = 0
          xycar_msg.speed = 0
          motor_pub.publish(xycar_msg)
          print("stop")  
       
    while (arData["DZ"] < 2):
        if(arData["DX"] < 0):
          xycar_msg.angle = -20
          xycar_msg.speed = -20
          motor_pub.publish(xycar_msg)
        else:
          xycar_msg.angle = 20
          xycar_msg.speed = -20
          motor_pub.publish(xycar_msg)             
    
    """
    for cnt in range(cnt):
        xycar_msg.angle = -ang
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
        
        if not ar_drive_flag:
          xycar_msg.angle = 0
          xycar_msg.speed = 0
          motor_pub.publish(xycar_msg)
          break
    """

def parking_dirve():
    global roll, pitch, yaw
    global arData
    global angle, speed
    global final_drive_flag
    global temp
    while(final_drive_flag and temp == 0):
       
      if abs(pitch) < 3 and abs(arData["DX"]) < 0.03:
          angle = 0
      
      # left
      if arData["DX"] <= -0.03 :
        #print("left")
        if arData["DX"] > -0.05:
            angle = -10
        else:
            angle = -20
            
        #if (pitch < -2 and arD
        ata["DZ"] < 0.5):
        #   back_drive2(50, 30)
         
              
      #right            
      if arData["DX"] >= 0.03 :
        #print("right")
        if arData["DX"] < 0.05:
            angle = 10
        else:
            angle = 20
        
        #if (pitch > 2 and arData["DZ"] < 0.5):
        #    back_drive1(50, 30)
      
      if (abs(pitch) > 3 and arData["DZ"] < 0.5):
        if pitch < 0:
          back_drive2(30, 40)
        else:
          back_drive1(30,40)
      
      
      if arData["DZ"] > 1.8:
          speed = 20
      elif arData["DZ"] > 1.0:
          speed = 15
      elif arData["DZ"] > 0.30:
          #print(arData["DZ"])
          speed = 10
          
      if abs(pitch) < 3 and arData["DZ"] <= 0.30:
          print("stop")
          final_drive_flag = False
          speed=0
      
      xycar_msg.angle = angle
      xycar_msg.speed = 0
      #xycar_msg.data = [angle, speed]
      motor_pub.publish(xycar_msg)


while not rospy.is_shutdown():
    #global xycar_msg
    #global arData
    #global ar_parking_flag, ar_drive_flag
    #global roll, pitch, yaw, temp
    flag1 = False
    flag2 = False
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
    print("angle : " + str(round(xycar_msg.angle,0)))
    print("speed : " + str(round(xycar_msg.speed,0)))
    print("id : " + str(temp))
    print("atan : " + str(atan))
    
    img = np.zeros((100,500,3))
    
    img = cv2.line(img,(25,65), (475,65),(0,0,255),2)
    img = cv2.line(img,(25,40), (25,90),(0,0,255),3)
    img = cv2.line(img,(250,40), (250,90),(0,0,255),3)
    img = cv2.line(img,(475,40), (475,90),(0,0,255),3)
    
    point = int(arData["DX"]) + 250
    
    if point > 475 :
        point = 475
    elif point < 25 :
        point = 25
        
    img = cv2.circle(img,(point, 65), 15, (0,255,0), -1)
    
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    cv2.putText(img, str(int(distance)), (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
    
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+ "DY:"+str(int(arData["DY"])) + "Yaw:"+str(round(yaw,1))
    cv2.putText(img, dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)    


    if ar_drive_flag:
      if not ar_parking_flag:
        ar_drive_flag = False
      left_drive(-50, 50)


    if ar_parking_flag:
      continue
    else:
      parking_dirve()
      
cv2.destrouAllWindows()
