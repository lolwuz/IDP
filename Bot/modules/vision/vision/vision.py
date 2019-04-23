import cv2
import numpy as np


class Vision:

    lower_blue = np.array([100, 160, 60])
    upper_blue = np.array([130, 255, 255])

    lower_red = np.array([160, 160, 60])
    upper_red = np.array([180, 255, 255])

    lower_green = np.array([38, 160, 60])
    upper_green = np.array([75, 255, 255])

    lower_yellow = np.array([22, 160, 60])
    upper_yellow = np.array([38, 255, 255])

    def __init__(self):

        self.camera = cv2.VideoCapture(0)

    def run(self):

        while (True):
            # read frame from camera
            ret, frame = self.camera.read()

            # resize for faster processing
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Threshold the HSV image to get only blue colors
            mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)
            mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
            mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

            # Bitwise-AND mask and original image
            res_blue = cv2.bitwise_and(frame, frame, mask=mask_blue)

            cv2.imshow('frame', frame)
            cv2.imshow('blue', mask_blue)
            cv2.imshow('red', mask_red)

            cv2.imshow("Cam", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


vision = Vision()
vision.run()


