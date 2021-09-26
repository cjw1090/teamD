#!/usr/bin/env python
#-- coding:utf-8 --

import rospy, time, cv2, math
import numpy as np

from xycar_msgs.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
turn_flag =True
class AR_PARKING:
    arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
    motor_pub = rospy.Publisher("/xycar_motor", xycar_motor, queue_size=1)
    xycar_msg = xycar_motor()
    

    def __init__(self):
        rospy.init_node('ar_parking', anonymous=False)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size=10)
        self.r=rospy.Rate(10)
        self.id=0

    def callback(self, msg):
        for i in msg.markers:
            self.id=i.id
            #print('id', self.id)

            self.arData["DX"] = i.pose.pose.position.x+0.1 if i.pose.pose.position.x>0 else i.pose.pose.position.x-0.1
            self.arData["DY"] = i.pose.pose.position.z
            self.arData["DZ"] = i.pose.pose.position.z

            self.arData["AX"] = i.pose.pose.orientation.x
            self.arData["AY"] = i.pose.pose.orientation.y
            self.arData["AZ"] = i.pose.pose.orientation.z
            self.arData["AW"] = i.pose.pose.orientation.w

    def back_drive(self):
        yaw=math.degrees(euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))[1])
        back=-yaw
        cnt=0

        while abs(yaw)>5.0:
            cnt+=1

            #print('first', yaw)
            self.xycar_msg.angle=back
            #print(self.id, self.xycar_msg.angle)
            self.xycar_msg.speed=-30
            self.motor_pub.publish(self.xycar_msg)

            yaw=math.degrees(euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))[1])

            self.r.sleep()

        print('finish first')

        for _ in range(cnt):
            self.xycar_msg.angle=back
            self.xycar_msg.speed=-30
            self.motor_pub.publish(self.xycar_msg)

            self.r.sleep()
        print('finish second')

        yaw=math.degrees(euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))[1])
        #print('second', yaw)
        while abs(yaw)>2.0:
            #print('second', yaw)
            self.xycar_msg.angle=-back
            self.xycar_msg.speed=-30
            self.motor_pub.publish(self.xycar_msg)

            yaw=math.degrees(euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))[1])

            self.r.sleep()
        print('finish third')

    def left_drive(self):
        #global xycar_msg, motor_pub
        #global ar_drive_flag
        #global angle, speed
        #global arData
        #print("go left--------------------------")
        
        """
        while (arData["DZ"] > 0.5):
            if(arData["DX"] < 0):
              xycar_msg.angle = -20
              xycar_msg.speed = 10
              motor_pub.publish(xycar_msg)
            else:
              xycar_msg.angle = 20
              xycar_msg.speed = 10
              motor_pub.p  ublish(xycar_msg)
        """
        global turn_flag
        for i in range(30):
              self.xycar_msg.angle = 0
              self.xycar_msg.speed = 0
              self.motor_pub.publish(self.xycar_msg)
              rospy.sleep(0.1)
              #print(self.arData["DZ"])
              #print("stop")  
           
        while (self.arData["DZ"] < 1.3):
            
            if(self.arData["DX"] < 0):
              self.xycar_msg.angle = 30
              self.xycar_msg.speed = -25
              self.motor_pub.publish(self.xycar_msg)
              #print("back1")
              #print(self.arData["DZ"])
            else:
              self.xycar_msg.angle = -30
              self.xycar_msg.speed = -25
              self.motor_pub.publish(self.xycar_msg)             
              #print("back2")
              #print(self.arData["DZ"])
        
        
        """         
        for _ in range(23):
            self.xycar_msg.angle = 50
            self.xycar_msg.speed = -30
            self.motor_pub.publish(self.xycar_msg)
            time.sleep(0.1)
        for _ in range(20):
            self.xycar_msg.angle = -10
            self.xycar_msg.speed = 10
            self.motor_pub.publish(self.xycar_msg)
            time.sleep(0.1)
        """
        for _ in range(25):
            self.xycar_msg.angle = -50
            self.xycar_msg.speed = 20
            self.motor_pub.publish(self.xycar_msg)
            time.sleep(0.1)
                    
        for _ in range(5):
            self.xycar_msg.angle = 0
            self.xycar_msg.speed = 0
            self.motor_pub.publish(self.xycar_msg)
            time.sleep(0.1)
        
            
        turn_flag = False
            
        









    def ar_parking(self):
        (roll, pitch, yaw)=euler_from_quaternion((self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]))
        #print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
	
        #roll = math.degrees(roll)
        #pitch = math.degrees(pitch)
        yaw = math.degrees(pitch)
        distance = math.sqrt(pow(self.arData["DX"],2) + pow(self.arData["DY"],2))
        #print(self.arData["DX"], self.arData["DY"])
        #print(self.arData["DZ"])
        atan = math.degrees(math.atan2(self.arData["DX"], self.arData["DY"]))

        #print('front', 'atan', atan, 'yaw', yaw, 'distance', distance)
        speed=15
        angle=atan
        global turn_flag
        
        if self.id==0:
            if(not turn_flag):
              print("parking_tag")
              if distance>2.0:
                  speed=0
  
              elif distance>1.8:
                  speed=0  
  
              elif distance>1.5:
                  speed=0
  
              elif distance>1.2:
                  speed=0
          
              else:
                  speed=0
  
                  #디버깅 요망
                  #print(yaw, atan, distance)
                  if abs(yaw)>8 and abs(atan)>8:
                      print('back')
                      self.back_drive()
                  else:
                      angle=0
        elif self.id==9:
            if(turn_flag):
              print("id 9")
              if self.arData["DZ"]>1.8:
                  speed=15
  
              elif self.arData["DZ"]>1.2:
                  speed=15  
  
              elif self.arData["DZ"]>0.8:
                  speed=15
  
              elif self.arData["DZ"]>0.4:
                  speed=10
                  print("check")
          
              else:
                  speed=0  
                  self.left_drive()
        #print(angle)               
        self.xycar_msg.angle=angle
        self.xycar_msg.speed=speed
        self.motor_pub.publish(self.xycar_msg)

        self.r.sleep()

if __name__ == '__main__':
    parking = AR_PARKING()
    
    while not rospy.is_shutdown():
        parking.ar_parking()
