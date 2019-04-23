import numpy as np
import cv2

greenLower = np.array([29,86,6])
greenUpper = np.array([64,255,255])

kernelOpen = np.ones((5, 5))
kernelClose = np.ones((20, 20))

def findMarker(image):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # edged = cv2.Canny(gray, 35, 125)

    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green = cv2.inRange(imgHSV, greenLower, greenUpper)

    greenOpen = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernelOpen)
    greenClose = cv2.morphologyEx(greenOpen, cv2.MORPH_CLOSE, kernelClose)

    _, conts, h = cv2.findContours(greenClose.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cv2.drawContours(image, conts, -1,(255,0,0), 3)

    for i in range(len(conts)):
        x,y,w,h = cv2.boundingRect(conts[i])
        cv2.rectangle(image,(x,y),(x+w, y+h), (0,0,255), 2)
        cv2.putText(image, "Green", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))


    # return cv2.minAreaRect(c)


def distanceToCamera(knownWidth, focalLength, perWidth):
    return (knownWidth * focalLength) / perWidth


knownDistance = 7.5

knownWidth = 7.5

imagePaths = ["block.png"]

image = cv2.imread(imagePaths[0])
marker = findMarker(image)
focalLength = (marker[1][0] * knownDistance) / knownWidth

for imagePath in imagePaths:
    image = cv2.imread(imagePath)
    marker = findMarker(image)
    cm = distanceToCamera(knownWidth, focalLength, marker[1][0])

    box = np.int0(cv2.boxPoints(marker))
    cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
    cv2.putText(image, "%.2fft" % (cm / 12),
                (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                2.0, (0, 255, 0), 3)
    cv2.imshow("image", image)
    cv2.waitKey(0)
