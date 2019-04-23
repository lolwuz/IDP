#!/usr/bin/env python
import math

import rospy
import numpy as np
import time
from robot_controller.msg import MotorState, PwmState
from sensor_msgs.msg import JointState
import cv2
from vision import picam
from base.base_node import BaseNode


class LineFollower(BaseNode):

    _WHEEL_BASE = 125
    _WHEEL_WIDTH = 140
    _MAX_ANGLE = 60
    _MAX_STEER_VELOCITY = 200
    _MAX_SUSPENSION_ANGLE = 47

    _DRIVING_JOINTS = {
        "steering_joint_la": "left",
        "steering_joint_ra": "right",
        "steering_joint_lb": "left",
        "steering_joint_rb": "right",
        "steering_joint_lc": "left",
        "steering_joint_rc": "right"
    }

    _SUSPENSION_JOINTS = {
        "suspension_joint_la": "left",
        "suspension_joint_ra": "right",
        "suspension_joint_lb": "left",
        "suspension_joint_rb": "right",
        "suspension_joint_lc": "left",
        "suspension_joint_rc": "right"
    }

    def __init__(self, name, is_running, rate):
        super(LineFollower, self).__init__(name, is_running, rate)
        rospy.logout("test")
        self.cam = picam.OpenCVCapture()
        self.been_on_line = False

        while not rospy.is_shutdown():
            if self.is_running:
                self.update()

    def update(self):
        super(LineFollower, self).update()

        frame = self.cam.read()
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

        isRed = self.SearchColor(redClose.copy(), frame)

        if not self.been_on_line:
            if isRed:
                self.change_pwm_state(0)
                self.change_motor_state("all", "off")

                self.been_on_line = True
                # stand still for 30 seconds
                print(str(self.been_on_line))
                time.sleep(30)

        self.change_motor_state("all", "up")
        self.change_pwm_state(500)

        print(contours)

        if len(contours) > 0:

            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)

            cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

            rospy.logerr(cx)

            if cx >= 120:
                #rospy.logerr("LEFT")
                self.steer(-0.05)

            if 120 > cx > 50:
                #rospy.logerr("Forward")
                self.steer(0)

            if cx <= 50:
                self.steer(0.05)
                #rospy.logerr("RIGHT")

            # self.line_follower_publisher.publish()



    def steer(self, percentage):
        # The minimal steering angle is 45 percent
        steer_angle_degrees = abs(self._MAX_ANGLE * percentage)
        steer_angle_radian = math.radians(steer_angle_degrees)

        radius_a = 0  # Inner wheel
        radius_b = 0  # Outer wheel

        multiplier = 0  # Multiplier 1 for left -1 for right
        if percentage > 0:
            radius_a = self._WHEEL_BASE / math.sin(steer_angle_radian)
            radius_b = radius_a + self._WHEEL_WIDTH * 2
            multiplier = 1
        elif percentage < 0:
            radius_b = self._WHEEL_BASE / math.sin(steer_angle_radian)
            radius_a = radius_b + self._WHEEL_WIDTH * 2
            multiplier = -1

        if radius_a == 0 or radius_b == 0:
            angle_right = 0
            angle_left = 0
        else:
            angle_right = math.asin(self._WHEEL_BASE / radius_a) * multiplier
            angle_left = math.asin(self._WHEEL_BASE / radius_b) * multiplier

        # Don't steer on minimal joystick.
        if abs(percentage) < 0.1:
            angle_right = 0
            angle_left = 0

        # Joint message
        joint_message = JointState()
        joint_message.header.stamp = rospy.Time.now()

        for name in self._DRIVING_JOINTS:
            joint_message.name.append(name)
            joint_message.velocity.append(200)
            joint_message.effort.append(0)

            if name == "steering_joint_la":
                joint_message.position.append(270 + math.degrees(angle_left))
            if name == "steering_joint_ra":
                joint_message.position.append(90 + math.degrees(angle_right))
            if name == "steering_joint_lb":
                joint_message.position.append(270)
            if name == "steering_joint_rb":
                joint_message.position.append(90)
            if name == "steering_joint_lc":
                joint_message.position.append(270 + math.degrees(-angle_left))
            if name == "steering_joint_rc":
                joint_message.position.append(90 + math.degrees(-angle_right))

        # Publish joint_state
        self.servo_steer_publisher.publish(joint_message)

    def SearchColor(self, colorClose, img):
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


if __name__ == '__main__':
    try:
        line_fol = LineFollower("line", True, 24)
    except rospy.ROSInterruptException:
        pass
