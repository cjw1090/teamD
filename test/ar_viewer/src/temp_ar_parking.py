#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


class PID():

    def __init__(self, kp, ki, kd):

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):

        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error


#xycar_msg = Int32MultiArray()

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0

def callback(msg):
    global arData
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        


rospy.init_node('ar_parking')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )
xycar_msg = xycar_motor()
#flag = False

def back_drive(ang,cnt):
    global xycar_msg, motor_pub
    print("back--------------------------")
    for cnt in range(cnt):
        xycar_msg.angle = ang
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
    for cnt in range(cnt):
        xycar_msg.angle = -ang
        xycar_msg.speed = -20
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)


while not rospy.is_shutdown():
    global xycar_msg
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
    print("angle : " + str(round(xycar_msg.angle,5)))
    print("speed : " + str(round(xycar_msg.speed,0)))
    
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
    
    y_value = arData["DZ"] * math.cos(pitch)
    x_value = -1 * arData["DZ"] * math.sin(pitch)
    #pi_value = math.atan(arData["DX"]/arData["DY"])
    error = x_value * 3700
    
    pid =PID(0.02, 0, 0.004) 
    angle = pid.pid_control(error)
            
    if pitch < -10 and arData["DX"] <-0.03 and arData["DZ"] < 0.7:
        back_drive(50, 20)
        
    if pitch > 10 and arData["DX"] > 0.03 and arData["DZ"] < 0.7:
        back_drive(50, 20)

    if arData["DZ"] > 1.8:
        speed = 20
    elif arData["DZ"] > 1.0:
        speed = 15
    elif arData["DZ"] > 0.30:
        print(arData["DZ"])
        speed = 10
        
    if abs(pitch) < 10 and arData["DZ"] <= 0.30:
        print("stop")
        speed=0
        
    # if (pitch > 20 or abs(arData["DX"] > 0.1)) and arData["DZ"] <0.30 :
    #     back_drive(-50,20)
    #     if flag1 == True:
    #         flag1=False
    # elif (pitch < -20 or abs(arData["DX"] > 0.1)) and arData["DZ"] <0.30 :
    #     back_drive(50,20)
    #     if flag2 == True:
    #         flag2=False
            
    

    xycar_msg.angle = angle
    xycar_msg.speed = speed
    #xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)
	

cv2.destrouAllWindows()
