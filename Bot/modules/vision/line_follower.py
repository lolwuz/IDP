#!/usr/bin/env python
import math

import rospy
import numpy as np
import time
from sensor_msgs.msg import JointState
import cv2



cam = cv2.VideoCapture(0)
been_on_line = False


def update(been_on_line):

    ret, frame = cam.read()
    crop_img = frame[60:120, 0:160]
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
    _, contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))
    imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    redLower = np.array([115, 128, 81])
    redUpper = np.array([186, 255, 255])
    red = cv2.inRange(imgHSV, redLower, redUpper)
    redOpen = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernelOpen)
    redClose = cv2.morphologyEx(redOpen, cv2.MORPH_CLOSE, kernelClose)

    isRed = SearchColor(redClose.copy(), frame)

    if not been_on_line:
        if isRed:
            #self.change_pwm_state(0)
            #self.change_motor_state("all", "off")

            been_on_line = True
            # stand still for 30 seconds
            print(str(been_on_line))
            #time.sleep(30)
            print("Sleep now")
    #self.change_motor_state("all", "up")
    #self.change_pwm_state(500)

    #print(contours)

    if len(contours) > 0:

        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
        cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)

        cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
        print(cx)

        if cx >= 120:
            print("LEFT")
            #self.steer(-0.05)

        if 120 > cx > 50:
            print("Forward")
            #self.steer(0)

        if cx <= 50:
            #self.steer(0.05)
            print("RIGHT")

        # self.line_follower_publisher.publish()

def SearchColor(colorClose, img):
    _, conts, h = cv2.findContours(colorClose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(conts) > 0:
        peri = cv2.arcLength(conts[0], True)
        approx = cv2.approxPolyDP(conts[0], 0.04 * peri, True)
        cnt = conts[0]
        cv2.drawContours(img, [cnt], -3, (255, 0, 0), 2)
        area = cv2.contourArea(cnt)
        for i in range(len(conts)):
            x, y, w, h = cv2.boundingRect(conts[i])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)
            return True

    return False


while(True):
    update(been_on_line)
    ret, frame = cam.read()
    cv2.imshow("c", frame)