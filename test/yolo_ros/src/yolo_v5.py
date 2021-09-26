#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,time
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor
#from sensor_msgs.msg import LaserScan
#from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
steering_pub = None
box_data = BoundingBoxes()
class_name = ''
motor_msg = xycar_motor()
lidar_points=None
ultra = [0,0,0,0,0]

def callback(msg):
  global box_data
  #print(msg)
  box_data = msg
  
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

def init_node_1():
  # 변수, sub, pub 선언
  global steering_pub
  rospy.init_node("human_follow")
  #rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
  rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
  rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
  rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)
  steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
  
  
def exit_node():
  #프로그램 끝날 때 할일 정리 
  print('finished')

  
def drive(angle, speed):
  # 모터제어 토픽을 발행하는 코드
  global steering_pub
  motor_msg.header.stamp = rospy.Time.now()
  motor_msg.angle = angle
  motor_msg.speed = speed
  #print(motor_msg)
  steering_pub.publish(motor_msg)
  
if __name__ == '__main__':
#def yolo_main(ar_id):
  # if ar_id = 5:
    #class_name = 'bicycle'
  #elif ar_id = 6:
    #class_name = 'pottedplant'
    
  global class_name, lidar_points
  angle = 0
  #class_name='pottedplant'
  init_node_1()
  class_name = 'bicycle'
  time.sleep(15)
  rate = rospy.Rate(10)
  while box_data is None:
    rate.sleep()
  while not rospy.is_shutdown():
    nobody = True
    boxes = box_data
    
    print(boxes.bounding_boxes)
    for i in range(len(boxes.bounding_boxes)):
      #print(boxes.bounding_boxes[i].Class)
      
      if boxes.bounding_boxes[i].Class == 'bicycle'  : 
        nobody = False
        #print("go")
        center = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
        #if class_name == 'bicycle':
        #   center = 640 - center
        angle = int(50.0 *((center - 320.0)/320.0))
        #if(ultra[0] < 30 or ultra[1] < 30) :
        
        if lidar_points == None:
          continue
        else:
          for i in range((int)(15*505)/360,(int)(45*505)/360):
            if (lidar_points[i] < 0.3):
              count +=1
              #print("find: ",i)
          
              if (count >= 5):
                drive(-angle,10)
            
          
          for i in range((int)(315*505)/360,(int)(345*505)/360):
            if (lidar_points[i] < 0.3):
              count +=1
              #print("find: ",i)
          
              if (count >= 5):
                drive(-angle,10)
          
        count =0
        

        #if class_name == 'bicycle'
          #angle = -angle
        #angle = -angle
       
      drive(angle, 10)
      #print(angle)
    if nobody:
      for _ in range(5):
        drive(-(angle), 10)
      for _ in range(3):
        drive(-angle, 10)
      print(-angle)
      #nobody = True
      rate.sleep()

  #rospy.spin()
  rospy.on_shutdown(exit_node)
