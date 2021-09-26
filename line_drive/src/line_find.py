import numpy as np
import cv2
import math
import time


class ParamFindHelper:

    def __init__(self, video_name, arrow_name):
        self.cap = cv2.VideoCapture(video_name)
        self.arrow_pic = cv2.imread(arrow_name, cv2.IMREAD_COLOR)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        
        self.last_time = time.time()
        self.roi_upper_y = 330
        self.roi_lower_y = 440
        self.guide_rect_y = 360
        self.steer_angle = 0
        self.candidate_lines = []
        self.lmr = {}
        self.target_point = (int(self.width/2), 0)
        self.cross_point_history = [(int(self.width/2), 0)]
        self.history_queue_size = 10
        self.target_fps = 100


    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


    def readFrame(self):
        ret, frame = self.cap.read()
        self.bgr_frame = frame

        return ret


    def showFrame(self):
        self.processGrayscale()
        self.processCanny()
        # set ROI
        self.canny_frame[:self.roi_upper_y, :] = 0
        self.canny_frame[self.roi_lower_y:, :] = 0

        self.processHough(20, 10)
        self.determineDirection()

        self.drawSteer()
        self.drawAuxiliaries()

        cv2.imshow("main", self.bgr_frame)


    def processGrayscale(self):
        self.grayscale_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_RGB2GRAY)


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
        angle_radian = math.atan(float(x-self.width/2) / (self.guide_rect_y - y))
        angle = math.degrees(angle_radian)
        angle = max(-50.0, angle)
        angle = min(angle, 50)
        self.steer_angle = -angle


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


    def drawSteer(self):
        origin_height = self.arrow_pic.shape[0]
        origin_width = self.arrow_pic.shape[1]
        steer_wheel_center = origin_height * 0.74
        arrow_height = self.height / 2
        arrow_width = (arrow_height * 462) / 728

        matrix = cv2.getRotationMatrix2D((origin_width / 2, steer_wheel_center), (self.steer_angle) , 0.7)
        arrow_pic = cv2.warpAffine(self.arrow_pic, matrix, (origin_width + 60, origin_height))
        arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_width, arrow_height), interpolation=cv2.INTER_AREA)

        gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

        arrow_roi = self.bgr_frame[arrow_height: self.height, (self.width / 2 - arrow_width / 2): (self.width / 2 + arrow_width / 2)]
        arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
        res = cv2.add(arrow_roi, arrow_pic)
        self.bgr_frame[(self.height - arrow_height): self.height, (self.width / 2 - arrow_width / 2): (self.width / 2 + arrow_width / 2)] = res


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




if __name__ == '__main__':

    pm = ParamFindHelper('kmu_track.mkv', 'steer_arrow.png')

    while pm.cap.isOpened():

        if not pm.readFrame():
            print("Unknown error!")
            break

        pm.showFrame()

        key = cv2.waitKey(1)

        if key == ord('q'):
            break
