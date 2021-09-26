#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import math
import time
import rospy, rospkg
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image


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

class ParamFindHelper:

    def __init__(self, video_name):
        #self.cap = cv2.VideoCapture(video_name)
        #self.arrow_pic = cv2.imread(arrow_name, cv2.IMREAD_COLOR)
        #self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        #self.cap = video_name
        self.bgr_frame = video_name
        
        self.width = 640
        self.height = 480
        self.last_time = time.time()
        self.roi_upper_y = 280
        self.roi_lower_y = 320    
        self.guide_rect_y = 360
        self.steer_angle = 0
        self.candidate_lines = []
        self.lmr = {}
        self.target_point = (int(self.width/2), 0)
        self.cross_point_history = [(int(self.width/2), 0)]
        self.history_queue_size = 10
        self.target_fps = 100
        
        #####
        self.lane_bin_th = 145
        self.warpx_margin = 20
        self.warpy_margin = 3
        self.warp_img_w = 320
        self.warp_img_h = 240

        self.warp_src  = np.array([
            [230-self.warpx_margin, 300-self.warpy_margin],  
            [45-self.warpx_margin,  450+self.warpy_margin],
            [445+self.warpx_margin, 300-self.warpy_margin],
            [610+self.warpx_margin, 450+self.warpy_margin]
        ], dtype=np.float32)

        self.warp_dist = np.array([
            [0,0],
            [0,self.warp_img_h],
            [self.warp_img_w,0],
            [self.warp_img_w, self.warp_img_h]
        ], dtype=np.float32)

        #####
        self.pid = PID(0.45, 0.0005, 0.25)
        ######

    def showFrame(self):
        self.bgr_frame = self.calibrate_image(self.bgr_frame)
        self.processGrayscale()
        self.processCanny()
        # set ROI
        self.canny_frame[:self.roi_upper_y, :] = 0
        self.canny_frame[self.roi_lower_y:, :] = 0
        cv2.imshow("canny_frame", self.canny_frame)
        #self.cali()
        #self.ff = self.canny_frame
        #self.f=self.calibrate_image(self.ff)
        #cv2.imshow("test", self.f)
        #cv2.imshow("tf_image", self.tf_image)
        self.processHough(20, 10)
        self.determineDirection()

        #self.drawSteer()
        self.drawAuxiliaries()
        self.drive()

        cv2.imshow("main", self.bgr_frame)

    def cali(self):
      calibrated = True
      if calibrated:
          self.mtx = np.array([
              [422.037858, 0.0, 245.895397], 
              [0.0, 435.589734, 163.625535], 
              [0.0, 0.0, 1.0]
          ])
          self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
          self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width, self.height), 1, (self.width, self.height))



    def calibrate_image(self, frame):
        self.cali()
        self.tf_image = cv2.undistort(frame, self.mtx, self.dist, None, self.cal_mtx)
        x, y, w, h = self.cal_roi
        self.tf_image = self.tf_image[y:y+h, x:x+w]
    
        return cv2.resize(self.tf_image, (self.width, self.height))
    
    def warp_image(img, src, dst, size):
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)
        self.warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)
    
        
        
    def processGrayscale(self):
        self.grayscale_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("grayscale_frame", self.grayscale_frame)

    def processCanny(self):
        blurred = cv2.GaussianBlur(self.grayscale_frame, (5, 5), 0)
        self.canny_frame = self.autoCanny(blurred, sigma=0.3)


    def processHough(self, minLineLength=100, maxLineGap=10):
        lines = cv2.HoughLinesP(self.canny_frame, 1, np.pi / 180, 50, 0, minLineLength, maxLineGap)

        if lines is not None:
            # transform as numpy array
            lines = np.array(lines)
            # reshape array: (len, 1, 4) -> (len, 4)
            lines = lines[:,0]
            # calculate slope
            lines = map(self.determineSlope, lines)
            # filter out None elements that were x1==x2
            self.candidate_lines = [x for x in lines if x is not None]
        else:
            self.candidate_lines = []


    def determineDirection(self):
        lines = self.candidate_lines

        # classify which area the line belongs to: left or middle or right
        # select representative line for each area
        lmr = self.findLmr(lines)
        self.lmr = lmr

        # first, try to find cross of left and right
        if 'left' in lmr and 'right' in lmr:
            cross_point = self.getCrossPoint(lmr['left'], lmr['right'])

        # if fails, try to find cross of left and middle
        elif 'left' in lmr and 'middle' in lmr:
            cross_point = self.getCrossPoint(lmr['left'], lmr['middle'])

        # if fails, try to find cross of middle and right
        elif 'middle' in lmr and 'right' in lmr:
            cross_point = self.getCrossPoint(lmr['middle'], lmr['right'])

        # if fails, use previous cross point
        elif len(lmr) == 1:
            x, y = self.cross_point_history[-1]
            x1, y1, x2, y2, a = lmr.values()[0]
            y = int(a * x + y1 - a * x1)
            cross_point = (x, y)
        else:
            cross_point = self.cross_point_history[-1]

        # push cross_point to history queue
        self.cross_point_history += [cross_point]
        if len(self.cross_point_history) > self.history_queue_size:
            self.cross_point_history = self.cross_point_history[1:]

        # moving average filter
        x, y = map(int, np.mean(self.cross_point_history, axis=0))
        self.target_point = (x, y)

        # avoid divide by zero
        if self.guide_rect_y == y:
            y = 0
        # calculate rotation angle of steering wheel
        #angle = x - self.width/2
        #angle = -(self.width/2 - x)
        '''
        angle_radian = math.atan(float(x-self.width/2) / (self.guide_rect_y - y))
        angle = math.degrees(angle_radian)
        angle = max(-50.0, angle)
        angle = min(angle, 50)
        self.steer_angle = angle
        '''

        #####
        error = (x - Width / 2)
        angle = (pid.pid_control(error))
        self.steer_angle = angle
        #####
        #angle = -(Width/2 - center)


    def drawAuxiliaries(self):
        # draw line candidates
        for line in self.candidate_lines:
            x1, y1, x2, y2, a = line
            cv2.line(self.bgr_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # draw lmr lines and rectangles
        for line in self.lmr.values():
            x1, y1, x2, y2, a = line
            scale = 1000
            guide_rect_x = int((self.guide_rect_y - y1 + a * x1) / a)
            cv2.line(self.bgr_frame, (int(x1-scale), int(y1-scale*a)), (int(x2+scale), int(y2+scale*a)), (255, 0, 0), 2)
            cv2.rectangle(self.bgr_frame, (guide_rect_x - 5, self.guide_rect_y - 5), (guide_rect_x + 5, self.guide_rect_y + 5), (0, 255, 0), 2)

        # draw a rectangle at viewport center
        cv2.rectangle(self.bgr_frame, (self.width / 2 - 5, self.guide_rect_y - 5), (self.width / 2 + 5, self.guide_rect_y + 5), (0, 0, 255), 2)

        # draw target point
        x, y = self.target_point
        cv2.rectangle(self.bgr_frame, (x - 5, y - 5), (x + 5, y + 5), (0, 255, 0), -1)
        cv2.rectangle(self.bgr_frame, (x - 5, self.guide_rect_y - 5), (x + 5, self.guide_rect_y + 5), (0, 255, 0), 2)

        # print steering wheel angle
        cv2.putText(self.bgr_frame, "angle: {:.5f}".format(-self.steer_angle), (0, self.height - 15), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))

        # print fps
        now_time = time.time()
        fps = 1 / (now_time - self.last_time)
        while fps > self.target_fps:
            now_time = time.time()
            fps = 1 / (now_time - self.last_time)
        self.last_time = now_time
        cv2.putText(self.bgr_frame, "fps: {:.5f}".format(fps), (0, self.height - 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))


    def autoCanny(self, image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)

        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        # return the edged image
        return edged


    def determineSlope(self, line):
        x1, y1, x2, y2 = line

        # ignore vertical line
        if x1 == x2:
            return None

        a = (y2 - y1) / float(x2 - x1)

        # ignore line of which slope is over or under threshold
        th = float(3)
        if abs(a) > th or abs(a) < 1/th:
            return None

        return (x1, y1, x2, y2, a)

    def findLmr(self, lines):
        lmr = {}

        for line in lines:
            x1, y1, x2, y2, a = line

            # find left line
            if a < 0 and max(x1, x2) < self.width * 3 / 10:
                left = lmr.get('left', (0, 0, 0, 0, 0))
                left_y = max(left[1], left[3])
                line_y = max(y1, y2)
                lmr['left'] = line if line_y > left_y else left

            # find right line
            elif a > 0 and min(x1, x2) > self.width*7/10:
                right = lmr.get('right', (0, 0, 0, 0, 0))
                right_y = max(right[1], right[3])
                line_y = max(y1, y2)
                lmr['right'] = line if line_y > right_y else right

            # find middle line
            elif (min(x1, x2) > self.width * 40 / 100) and (max(x1, x2) < self.width * 60 / 100):
                middle = lmr.get('middle', (0, 0, 0, 0, 0))
                middle_y = max(middle[1], middle[3])
                line_y = max(y1, y2)
                lmr['middle'] = line if line_y > middle_y else middle

        return lmr

    def getCrossPoint(self, l, r):
        x = ((l[1] - r[1]) + (r[4]*r[0] - l[4]*l[0])) / (r[4] - l[4])
        y = l[4] * x + l[1] - l[4] * l[0]
        return int(x), int(y)


    # publish xycar_motor msg
    def drive(self):
        global pub

        msg = xycar_motor()
        msg.angle = self.steer_angle
        msg.speed = 10
        print(self.steer_angle)

        pub.publish(msg)

pub = None
bridge = CvBridge()
base_image = np.empty(shape=[0])

def img_callback(data):
    global base_image   
    base_image = bridge.imgmsg_to_cv2(data, "bgr8")



if __name__ == '__main__':

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    #print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(0.1)
    #pid = PID(0.5, 0.0005, 0.05)
    while True:
        while not base_image.size == (640*480*3):
            continue
        
        #cv2.imshow("base_image", base_image)
        pm = ParamFindHelper(base_image)

        pm.showFrame()
        #error = (center - Width / 2)
        #angle = (pid.pid_control(error))
        #pm.drive(angle)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    
    
