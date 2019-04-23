import cv2
from matplotlib import pyplot as plt
#from picamera import picamera
import numpy as np

#cam = PiCamera()
cam = cv2.VideoCapture(0)
#rawCapture = PiRGBArray(camera)

KNOWN_DISTANCE = 7.5
KNOWN_PIXELS = 2465
KNOWN_WIDTH = 6.5

#75mm = 2465 pixels (6.5cm)


def SearchCup(img):
    """
    Searching for the cup and outline it
    :param img: frame of video
    :return:
    """

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray = cv2.blur(gray,(3,3))

    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_TOZERO)#1)
    #cny = CannyImage(img, gray)

    cny = cv2.Canny(gray, 100, 255, cv2.THRESH_BINARY)

    _, contours, h = cv2.findContours(cny, 1, 2)


    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1.2, 100)

    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(img, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)



    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)
        #print str(len(approx)) + " " + str(area)

        if ((len(approx) > 8) & (area > 30)):
            #check for an area bigger than 400 so it skips over certain objects that are too small
            if(area > 400):
                cv2.drawContours(img, [cnt], 0, (0,0,255), -1)
                if len(contours) > 0:
                    (_, cnts, _) = cv2.findContours(cny.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    #cv2.drawContours(img, cnts, -1, (127, 127, 127), 2)
                    c = max(cnts, key=cv2.contourArea)
                    # print str(cv2.minAreaRect(c))
                    marker = cv2.minAreaRect(c)
                    #focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
                    distance = DefineDistanceToCup(KNOWN_WIDTH, focalLength, marker[1][0])

                box = np.int0(cv2.boxPoints(marker))
                cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
                cv2.putText(img, str(((distance) / 12) * 30.48) + "cm",
                            (box[0][1], box[1][1]),  # (img.shape[1] - 200, img.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2.0, (0, 255, 0), 3)

        if len(approx) == 4 & (area > 10):
            print(area)
            cv2.drawContours(img, [cnt], 0, (255, 0, 0), 4)

        # if len(approx) == 5:
        #     print "pentagon"
        #     cv2.drawContours(img, [cnt], 0, 255, -1)
        # elif len(approx) == 3:
        #     print "triangle"
        #     cv2.drawContours(img, [cnt], 0, (0, 255, 0), -1)
        # elif len(approx) == 9:
        #     print "half-circle"
        #     cv2.drawContours(img, [cnt], 0, (255, 255, 0), -1)
        # elif len(approx) > 8 & (area > 30):
        #     print "circle"
        #     cv2.drawContours(img, [cnt], 0, (0, 255, 255), -1)

    cv2.imshow("hi", thresh)
    cv2.imshow("canny", cny)

    # ret, th = cv2.threshold(grayblur, 180, 255, cv2.THRESH_BINARY_INV)
    # image, contours, hierarchy = cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #
    # cv2.drawContours(img, contours, -1, (150,50, 100), 3)
    #
    # hierarchy = hierarchy[0]
    #
    # for cnr in range(len(contours)):
    #     cnt = contours[cnr]
    #
    #     #print(cnt)

#F = (P x  D) / W
#F = Focal distance
#P = Pixels of object
#D = Distance to object from camera
#W = Width of object
#F = (2465 x 75mm) / 65mm
def DefineDistanceToCup(knownWidth, focalLength, acWidth):

    #print(str(knownWidth) + " * " + str(focalLength) + " / " + str(acWidth) + " = " + str(((((knownWidth * focalLength) / acWidth) / 12) * 30.48)))
    return ((((knownWidth * focalLength) / acWidth) / 12) * 30.48)


def CannyImage(image, color):
    imgHsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    edged = cv2.Canny(color, 35, 125)

    return edged


def find_marker(image):
    # convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    # compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)


imagepath = "cupimg_d75mm.jpg"
image = cv2.imread(imagepath)
marker = find_marker(image)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

while(True):
    ret, img = cam.read()
    #camera.capture(rawCapture, format="bgr")
    #img = rawCapture.array

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(imgHSV)
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayblur = cv2.GaussianBlur(grayscale, (5,5), 0)



    ret, thresh1 = cv2.threshold(grayscale, 50, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5,5),np.uint8) #square image kernel used for erosion
    erosion = cv2.erode(thresh1, kernel,iterations = 1) #refines all edges in the binary image
    opening = cv2.morphologyEx(erosion, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE,
                               kernel)  # this is for further removing small noises and holes in the image

    #Canny = CannyImage()
    SearchCup(img)

    # cv2.imshow("h", h)
    # cv2.imshow("s", s)
    # cv2.imshow("v", v)


    cv2.imshow("gray", grayblur)
    cv2.imshow("Cam", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
