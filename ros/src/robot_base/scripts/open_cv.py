#!/usr/bin/env python
import rospy
import cv2
from vision.picam import OpenCVCapture
from distutils.version import LooseVersion



def test():

    if LooseVersion(cv2.__version__).version[0] == 2:
        pass
        # Whatever OpenCV2 code
    else:
        cam = OpenCVCapture()
        img = cam.read()
        # Load an color image in grayscale
        #img = cv2.imread('/home/pi/ros/src/robot_base/scripts/mnangagwa.jpeg', 0)
        cv2.imshow('image', img)
        cv2.waitKey(0)

# Whatever OpenCV3 code


if __name__ == "__main__":
    test()
