#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32MultiArray
from tf.transformations import euler_from_quaternion

lidar_points = None
ar_flag=0
prev=0
ultra = [0,0,0,0,0]
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

def ultra_callback(msg):
    global ultra

    for j in msg.data:
    #print(msg.data)
    #print(msg.data[0])
    #left
        ultra[0] = msg.data[0]
    #right
        ultra[1] = msg.data[4]
    #rear right
        ultra[2] = msg.data[5]
    #rear mid
        ultra[3] = msg.data[6]
    #rear left
        ultra[4] = msg.data[7]

def lidar_callback(data):
    global lidar_points

    lidar_points = data.ranges

def ar_callback(msg):
    global ar_flag, arData

    for i in msg.markers:
        if prev==0 and i.id==1:
            ar_flag=1
        elif prev == 1 and i.id == 2 :
            ar_flag = 2

        elif prev == 2 and i.id == 3:
            ar_flag = 3
            
        x1 = i.pose.pose.position.x
        if x1 > 0 : 
            arData["DX"] = x1 + 0.1
        else:
            arData["DX"] = x1 - 0.1
        #arData["DX"] = x + 0.1 if x1 > 0 else x1 - 0.1
    arData["DY"] = i.pose.pose.position.z
            	#self.arData["DZ"] = i.pose.pose.position.z
    arData["AX"] = i.pose.pose.orientation.x
    arData["AY"] = i.pose.pose.orientation.y
    arData["AZ"] = i.pose.pose.orientation.z
    arData["AW"] = i.pose.pose.orientation.w

	#ar_flag=1

pub = None
rate = None
"""
rospy.init_node('my_lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback)
rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)

pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
data = xycar_motor()
rate = rospy.Rate(10)
"""
data = xycar_motor()
angle=0


def drive_180():
    global data, pub, rate

    #print("180--------------------------")
    while True:
	#print(ultra[2], ultra[3], ultra[4])
        if(ultra[2]<= 20 or ultra[3] <=20 or ultra[4] <=20):
          #print("obstacle!")

            break

        data.angle = 50
        data.speed = -30
        pub.publish(data)

        rate.sleep()

    while ar_flag==2:		
        data.angle = -50
        data.speed = 15
        pub.publish(data)
        
	rate.sleep()

def back_drive():
    global data, pub, rate

    yaw=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))[1]
    back=math.degrees(-yaw)

    while abs(yaw)>0.02:
        data.angle=back
        data.speed=-30
            
        pub.publish(data)
        yaw=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))[1]
             
        rate.sleep()

        #print('finish first')

    while abs(back)-abs(math.degrees(yaw))>1.0:
        data.angle=back-math.degrees(yaw)
        data.speed=-30

        pub.publish(data)
        yaw=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))[1]

        rate.sleep()

        #print('finish second')

    while abs(yaw)>0.02:
        data.angle=-back
        data.speed=-30
        
        pub.publish(data)
        yaw=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))[1]

        rate.sleep()

        #print('finish third')

def ar_follow():
    global angle, data, ar_flag, pub, rate


    (roll, pitch, yaw)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    yaw = math.degrees(pitch)

    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    atan = math.degrees(math.atan2(arData["DX"], arData["DY"]))
    angle=atan

    speed=15
    #print(distance)
    if distance>1.8:
        algorithm2xycar()
            
        return

    elif distance>0.9:
        speed=12    

    elif distance>0.6:
        speed=10

    elif distance>0.3:
        speed=9
        
    else:
        speed=0

        if abs(yaw)>5 and abs(atan)>25:
                #print('drive_180')

	        drive_180()

    data.angle=angle
    data.speed=speed
    pub.publish(data)

    rate.sleep()

def algorithm2xycar():
    global data, angle, pub, rate
	
    sensor_value = np.concatenate((lidar_points[380:], lidar_points[0:125])) 
    l=[s for s in sensor_value[100:150] if s>0.0]
    if not l:
        for i in range(10):
            #print('for1', i)
            data.angle=angle
            data.speed=-30

            pub.publish(data)
            rate.sleep()

        return

    m = min(l)
	#n = max(l)
	#print('min', m, 'max', n)
        #print('min1', m)
    idx=np.concatenate((np.arange(270, 360, 0.72), np.arange(0, 90, 0.72)))
    cos_list=[math.cos(math.radians(d)) for d in idx]
    #print('cos list', cos_list)
    sin_list=[math.sin(math.radians(d)) for d in idx]
    #print('sin list', sin_list)	

    avg_x=0
    avg_y=0

    for i in range(len(idx)):
        avg_x+=sin_list[i]*float(sensor_value[i])
        avg_y+=cos_list[i]*float(sensor_value[i])

        avg_x=float(avg_x)/5.0
        avg_y=float(avg_y)/5.0
        print(avg_x, avg_y)
        avg_distance=float(math.sqrt(float(avg_x**2)+float(avg_y**2)))
        angle=math.degrees(math.asin(avg_x/avg_distance))

        if angle>50:
            angle=50.0
        elif angle<-50:
            angle=-50.0

    if m<=0.3:
        while True:
            sensor_value=np.concatenate((lidar_points[380:], lidar_points[0:125]))
            l=[s for s in sensor_value[100:150] if s>0.0]
            if not l:
                for i in range(20):
                    #print('for2', i)
                    data.angle=angle
                    data.speed=-30
        #print('data3', data)

                    pub.publish(data)
                    rate.sleep()

                return
            m=min(l)
            #print('min2', m)
            if m>0.6:
                return

            data.speed=-30
            data.angle=angle
            #print('data2', data)

            pub.publish(data)
            rate.sleep()
        else:
	          data.angle=-angle
            data.speed=15
            
        #print('data1', data)
        pub.publish(data)
	rate.sleep()

def dqn_lidar_main():
    global prev, pub, rate,data, ar_flag
 
    #rospy.init_node('my_lidar')
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rate = rospy.Rate(10)

#    if ar_flag==0:
#	    continue

    if ar_flag==1:
	    if lidar_points != None:
		    

		#print('flag', ar_flag)
	      dist=algorithm2xycar()
	      prev=1
    
    elif ar_flag==2:
        #print('flag', ar_flag)
	    ar_follow()
	    prev=2

    elif ar_flag==3:
	    if lidar_points!=None:
	
	      algorithm2xycar()
	      prev=3
