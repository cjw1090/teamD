#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy,time
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor

#from ar_track_alvar_msgs.msg import AlvarMarkers

steering_pub = None
box_data = BoundingBoxes()
class_name = ''
motor_msg = xycar_motor()
#yolo_flag = False
yolo_flag_1 = False
yolo_flag_2 = False
person = None

def callback(msg):
  global box_data
  #print(msg)
  box_data = msg
'''  
def ar_callback(mag):
    global yolo_flag, class_name
    for i in msg.markers:
        if (i.id == 5) :
          yolo_flag = True
          class_name = 'bicycle'
          print(class_name)
          break
        elif i.id == 6 :
          yolo_flag = True
          class_name = 'pottedplant'
          break
        #print("hihihihihih")
'''
def init_node_1():
  # 변수, sub, pub 선언
  global steering_pub
  rospy.init_node("human_follow")
  #rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
  rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
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
  global yolo_flag
  global class_name
  angle = 0
  #class_name='pottedplant'
  init_node_1()
  class_name = 'pottedplant'
  time.sleep(15)
  rate = rospy.Rate(10)
  while box_data is None:
    rate.sleep()
  while not rospy.is_shutdown():
    nobody = True
    boxes = box_data
    #print(boxes)
    #print(
    #print(len(boxes.bounding_boxes), boxes.bounding_boxes)
    #print(boxes)

    for i in range(len(boxes.bounding_boxes)):
      #print(boxes.bounding_boxes[i].Class)
      
      if boxes.bounding_boxes[i].Class == class_name : 
      #if boxes.bounding_boxes[i].Class == class_name :
        nobody = False
        print("go")
        center = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
        angle = int(50.0 *((center - 320.0)/320.0))
        #for _ in range(3):
        #if class_name == "pottedplant":
        #  angle = 50
        #elif class_name == "bicycle":
        #  angle = -50
        
        #for _ in range(30):
        #    print(angle)
        #    drive(angle,20)
        #    rate.sleep()
        #for _ in range(5):
        #    drive(0 , 10)
        #    rate.sleep()
        #rate.sleep()
        #for _ in range(5):
        #  drive(angle,15)
        #for _ in range(10):
        #  drive(0, 15)
        
        #drive(angle,10)
        
        for _ in range(13):
          drive(angle, 10)
          rate.sleep()
        for _ in range(3):
          #drive(-(angle//2), 10)
          drive(-angle, 10)
          rate.sleep()
        '''
        if not yolo_flag_1:
          yolo_flag_1 = True
        else:
          yolo_flag_2 = True
        #yolo_flag_1 = True
        '''
          # 여기서 ar 인식해야할듯
    #if yolo_flag_2:
        #break
        #break
        #rospy.on_shutdown(exit_node)
    if nobody:
      #print(nobody)
      '''
      if yolo_flag_1 :
        for _ in range(3):
          drive(-angle , 10)
        for _ in range(5):
          drive(-angle , 10)
      else:
      '''
      for _ in range(3):
        drive(-angle,10)
      for _ in range(5):
        drive(-angle, 0)
      #nobody = True
      rate.sleep()

  #rospy.spin()
  rospy.on_shutdown(exit_node)

