#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal
import queue

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
#Width = 1280
#Height = 720
#Offset = 340
Offset = 280
Gap = 40
 

#########################################################################################################
center_que=[]
left_que=[]
right_que=[]
total_l=0
total_r=0
total =0
lane_bin_th = 145
warpx_margin = 20
warpy_margin = 3
warp_img_w = 320
warp_img_h = 240

warp_src  = np.array([
    [230-warpx_margin, 300-warpy_margin],  
    [45-warpx_margin,  450+warpy_margin],
    [445+warpx_margin, 300-warpy_margin],
    [610+warpx_margin, 450+warpy_margin]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))
    
def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv
#########################################################################################################





def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub
    print(Speed)
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 255), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0
    x1_sum = 0.0
    y1_sum = 0.0
    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        x1_sum += x1
        y1_sum += y1
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg
    
    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)
        angle_radian = math.atan(float(x-Width/2) / (self.guide_rect_y - y))
        angle = math.degrees(angle_radian)
        angle = max(-50.0, angle)
        angle = min(angle, 50)
        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global lane_bin_th
    global total_l, total_r
    global left_que, right_que
    

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(frame,(kernel_size, kernel_size), 0)
    
    ###################################################################
    # binary
    #_, L, _ = cv2.split(cv2.cvtColor(blur_gray, cv2.COLOR_BGR2HLS))
    #_, lane = cv2.threshold(L, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #_, lane = cv2.threshold(L, 145, 255, cv2.THRESH_BINARY)
    ###################################################################
    
    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(lane), low_threshold, high_threshold)
    cv2.imshow('edge', edge_img)
    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
    
    #######################################################   
    #left_que.append(lpos)
    #print(lpos)
    #que_len1=len(left_que)
    #if que_len1 > 10:
    #  left_que.pop(0)
    #  que_len1 = que_len1 -1
    
    
    #print(que_len1)           
    #for i in range(0, que_len1):
    #  total_l = total_l + left_que[i]
        
    #lpos = total_l/que_len1
        
        
        
    #right_que.append(rpos)
    #que_len2=len(right_que)
    #if que_len2 > 10:
    #  right_que.pop(0)
    #  que_len2 = que_len2 -1
        
        
    #for i in range(0, que_len2):
    #  total_r = total_r + right_que[i]
        
    #rpos = total_r/que_len2
                
        
    #######################################################
    
    
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)
    
    # show image
    cv2.imshow('result', frame)
    
    total_l=0
    total_r=0        

    return lpos, rpos

###############################################################
class PID():
    def __init__(self, kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_error=0.0
        self.i_error=0.0
        self.d_error=0.0
        
    def pid_control(self, cte):
        self.d_error = cte-self.p_error
        self.p_error=cte
        self.i_error+=cte
        
        return self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error
###############################################################


def start():
    global pub
    global image
    global cap
    global Width, Height
    global total
    global center_que

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    #print "---------- Xycar A2 v1.0 ----------"
    #rospy.sleep(2)

    while True:
        while not image.size == (640*480*3):
            continue
        ####################################
        cali_image = calibrate_image(image)
        #warp_img, M, Minv = warp_image(cali_image, warp_src, warp_dist, (warp_img_w, warp_img_h))
        
        ####################################
        lpos, rpos = process_image(cali_image)

        center = (lpos + rpos) / 2
        
        """
        center_que.append(center)
        que_len=len(center_que)
        if que_len >20:
          center_que.pop()
          que_len = que_len -1
        
        
        for i in range(0, que_len):
          total = total + center_que[i]
        
        center_final = total/que_len
        """
        angle_radian = math.atan(float(x-Width/2) / (self.guide_rect_y - y))
        angle = math.degrees(angle_radian)
        angle = max(-50.0, angle)
        angle = min(angle, 50)
        
        error = (center - Width/2)
        angle = -(Width/2 - center)*0.8
        
        #if angle > 50 :
        #    angle = min(50, angle)
        #elif angle < -50:
        #    angle = max(-50, angle)
        #print(angle) 
        pid = PID(0.56,0.00005,0.12)
        angle = (pid.pid_control(error))
        drive(angle, 15)
        #total =0

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    start()
