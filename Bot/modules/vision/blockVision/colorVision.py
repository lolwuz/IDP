import cv2
import numpy as np
import math
import Vision

# setup lower and upper bound
greenLower = np.array([45,84,51])
greenUpper = np.array([89,255,255])

redLower = np.array([115, 128, 81])
redUpper = np.array([186, 255, 255])

blueLower = np.array([57, 153, 56])
blueUpper = np.array([150, 255, 255])

yellowLower = np.array([23, 87, 141])
yellowUpper = np.array([161, 255, 255])

orangeLower = np.array([0, 50, 80])
orangeUpper = np.array([20, 255, 255])

# initialize camera
cam = cv2.VideoCapture(0)

kernelOpen = np.ones((5, 5))
kernelClose = np.ones((20, 20))

knownDistance = 7.5
knownWidth = 7.5
focalLength = 0

def test():
    print("a: " + str(1.5 % 2.5) + " b: " + str(1.5 % 7.5) + " c: " + str(2.5 % 7.5))


def SearchColor(colorname, colorClose, img):
    _, conts, h = cv2.findContours(colorClose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    if len(conts) > 0:
        peri = cv2.arcLength(conts[0], True)
        approx = cv2.approxPolyDP(conts[0], 0.04 * peri, True)
        #print(str(len(approx)))
       # print(len(conts))
        #if len(approx) == 4:
        #    if len(conts) == 1:
        cnt = conts[0]
        # for i in range(len(cnt)):
        # print("contour: " + str(i) + " " + "x:" + str(cnt[i][0][1]) + " y:" + str(cnt[i][0][0]))
        # print("size of conts " + str(i) + "  " + str(len(cnt)))

        # print("largest X " + str(BoundaryContours(cnt)[0]) + " smallest X " + str(BoundaryContours(cnt)[1]))

        cv2.drawContours(img, [cnt], -3, (255, 0, 0), 2)
        area = cv2.contourArea(cnt)
        for i in range(len(conts)):
            x, y, w, h = cv2.boundingRect(conts[i])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)
            cv2.putText(img, colorname, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
            cv2.putText(img, FindSide(w, h)[0] + " " + FindSide(w, h)[1], (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255))



def findMarker(image, color):
    imgHsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    edged = cv2.Canny(color, 35, 125)

    _, cnts, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    #print str(cnts)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        #print str(cv2.minAreaRect(c))
        return cv2.minAreaRect(c)

    return []

def CannyImage(image, color):
    imgHsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    edged = cv2.Canny(color, 35, 125)

    return edged

def CalculateDistanceBoundary(img, marker):
    if len(marker) > 0:
        inches = distance_to_camera(knownWidth, focalLength, marker[1][0])

        # draw a bounding box around the image and display it
        box = np.int0(cv2.boxPoints(marker))
        cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
        #print(str(box))
#        cv2.putText(img, FindSide(w, h)[0] + " " + FindSide(w, h)[1], (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255))
        cv2.putText(img, "%.2fcm" % ((inches / 12) * 30.48),
                    (box[0][1],box[1][1]),#(img.shape[1] - 200, img.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2.0, (0, 255, 0), 3)

        iteration = 0
        for vertexes in box:
            #print(str(vertexes[0]))
            cv2.putText(img, str(iteration), (int(vertexes[0]), int(vertexes[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255))
            iteration = iteration + 1

        #sideStr = FindSide(DistanceBetweenVector2(box[0], box[2]),DistanceBetweenVector2(box[0], box[1]))
        #print(str(DistanceBetweenVector2(box[0], box[2])) + " " + str(DistanceBetweenVector2(box[0], box[1])))
        #print(sideStr)
        #cv2.putText(img, str(sideStr[0] + " " + sideStr[1]), (box[0][0], box[0][1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255))
        pos = sum(box) / len(box)
        cv2.circle(img, (pos[0], pos[1]), 1, (255,255,255), 2) #middle of block

def CalculateFace(division):
    global knownDistance
    global knownWidth
    knownWidth = 7.5 / division

imagePaths = ["block1.png", "block2.png", "block3.png"]
image = cv2.imread(imagePaths[0])
imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
green = cv2.inRange(imgHSV, greenLower, greenUpper)
greenOpen = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernelOpen)
greenClose = cv2.morphologyEx(greenOpen, cv2.MORPH_CLOSE, kernelClose)
marker = findMarker(image, greenClose)
focalLength = (marker[1][0] * knownDistance) / knownWidth

def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    #print(str(knownWidth))
    return (knownWidth * focalLength) / perWidth


def DistanceBetweenVector2(A, B):
    v0 = B[0] - A[0]
    v1 = B[1] - A[1]
    return math.sqrt(v0*v0 + v1*v1)


def BoundaryContours(cnt):
    smallestX = 1000
    largestX = 0

    for i in range(len(cnt)):
        if largestX < cnt[i][0][1]:
            largestX = cnt[i][0][1]
        elif smallestX > cnt[i][0][1]:
            smallestX = cnt[i][0][1]
        else:
            continue

    return [largestX, smallestX]

def FindSide(w, h):
    rstr = ""  # returnstring
    sstr = ""  # sidestring
    ostr = ""  # orientationstring

    # 1.5x2.5x7.5

    # Find orientation of block, Flat or Straight
    if w > h:
        ostr = "F"  # + "(" + str(w / h) + " | " + str(h / w) +")"
        # ostr = "F"
    else:
        # ostr = "S"
        ostr = "S"  # + "(" + str(w / h) + " | " + str(h / w) +")"

    # Find side of the block
    if (w / h) == 1 or (h / w) == 1:
        sstr = "A"

    elif (w / h) == 2 or (h / w) == 2:
        sstr = "C"


    elif (w / h) == 4 or (h / w) == 4:
        sstr = "B"


    else:
        sstr = "-"

    if ostr == "F" and sstr == "A":
        CalculateFace(3)

    elif ostr == "S" and sstr == "A":
        CalculateFace(5)

    elif ostr == "S" and sstr == "B":
        CalculateFace(3)

    elif sstr == "B" and  ostr == "F":
        CalculateFace(1)

    elif sstr == "C" and ostr == "S":
        CalculateFace(3)

    elif sstr == " C" and ostr == "F":
        CalculateFace(1)


    # if ((h * 2) > h) and ((h*2) -h) < 20:
    #    ostr = "1.5"

    # if w > h and (h * 3) < w:
    #     return "Green bottomside flat" + str(h*3) + "/" + str(w)
    #
    # elif h > w and (w * 3) < h:
    #     return "Green bottomside straight" + str(h*3)
    #
    # elif w > h and (h *2) > w:
    #     return "Green flatside flat" + str(h * 2) + " / " + str(w)
    #
    # elif h > w and (w * 2) < h:
    #     return "Green flatside straight" + str(h * 2) + " / " + str(w)
    #
    # elif w > h and (h * 2) > w:
    #     return "Green underside flat" + str(h*2) + " / " + str(w)
    #
    # elif h > w and (h * 2) > w:
    #     return "Green underside straight" + str(h * 2) + " / " + str(w)

    return [sstr, ostr]#rstr + " " + sstr + " " + ostr  # + " " + str(w) + " / " + str(h)

while (True):
    # read frame from camera
    ret, img = cam.read()

    # resize for faster processing
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    

    # mask for the color
    green = cv2.inRange(imgHSV, greenLower, greenUpper)
    red = cv2.inRange(imgHSV, redLower, redUpper)
    blue = cv2.inRange(imgHSV, blueLower, blueUpper)
    yellow = cv2.inRange(imgHSV, yellowLower, yellowUpper)
    orange = cv2.inRange(imgHSV, orangeLower, orangeUpper)

    greenOpen = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernelOpen)
    greenClose = cv2.morphologyEx(greenOpen, cv2.MORPH_CLOSE, kernelClose)

    redOpen = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernelOpen)
    redClose = cv2.morphologyEx(redOpen, cv2.MORPH_CLOSE, kernelClose)

    blueOpen = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernelOpen)
    blueClose = cv2.morphologyEx(blueOpen, cv2.MORPH_CLOSE, kernelClose)

    yellowOpen = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernelOpen)
    yellowClose = cv2.morphologyEx(yellowOpen, cv2.MORPH_CLOSE, kernelClose)

    orangeOpen = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernelOpen)
    orangeClose = cv2.morphologyEx(orangeOpen, cv2.MORPH_CLOSE, kernelClose)

    # Track colors
    SearchColor("Green", greenClose.copy(), CannyImage(img, greenClose))
    SearchColor("Red", redClose.copy(), img)
    SearchColor("Blue", blueClose.copy(), img)
    SearchColor("Yellow", yellowClose.copy(), CannyImage(img, yellowClose))#img)
    SearchColor("Orange", orangeClose.copy(), img)

    marker = findMarker(img, greenClose)
    CalculateDistanceBoundary(img, marker)
    marker = findMarker(img, redClose)
    CalculateDistanceBoundary(img, marker)
    marker = findMarker(img, blueClose)
    CalculateDistanceBoundary(img, marker)
    marker = findMarker(img, yellowClose)
    CalculateDistanceBoundary(img, marker)
    marker = findMarker(img, orangeClose)
    CalculateDistanceBoundary(img, marker)

    a = CannyImage(img, yellowClose)

    cv2.imshow("canny", a)
    cv2.imshow("Cam", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
