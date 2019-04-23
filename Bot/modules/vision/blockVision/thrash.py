import numpy as np
import cv2


class Video:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.previousFrames = []
        self.windowFrame = {}  # Save the windows made in createNewWindow
        self.paused = False
        self.frameCount = 0

    def createNewWindow(self, name, **kwargs):
        """
         Args:
             name: name of the window
         Kwargs:
             frame, xPos, yPos
         """
        frameForWindow = kwargs.get("frame", self.frame)
        if frameForWindow is None:
            self.getVideo()
            frameForWindow = self.frame  # First window opened if there are no windows opened

        xPos = kwargs.get("xPos", 20)
        yPos = kwargs.get("yPos", 20)

        # Create and setup window
        cv2.namedWindow(name)
        cv2.moveWindow(name, xPos, yPos)
        self.windowFrame[name] = frameForWindow[1]

    def isCameraConnected(self):
        return self.cap.isOpened()

    def getVideo(self):
        if not self.paused:
            ret, newFrame = self.cap.read()
            try:
                self.frame = newFrame.copy()
            except:
                print ("Frame not captured")
                self.previousFrames.append(self.frame.copy())
                if len(self.previousFrames) > 10:
                    del self.previousFrames[0]

                if self.frameCount >= 100:  # Keeps the framecount to under 100
                    self.frameCount = 0
                else:
                    self.frameCount += 1

    def display(self, window, **kwargs):
                """
                Args:
                    window: The string name of the window to display the image
                KWARGS:
                    "frame" : The frame to display.
                """
                cv2.imshow(window, self.windowFrame[window])


###################################################################################################################

import numpy as np
import cv2


class Video:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)

        self.kernelOpen = np.ones((5, 5))
        self.kernelClose = np.ones((20, 20))

        self.greenLower = np.array([46, 72, 62])
        self.greenUpper = np.array([86, 255, 255])

        self.redLower = np.array([127, 43, 0])
        self.redUpper = np.array([186, 255, 255])

        self.blueLower = np.array([88, 131, 167])
        self.blueUpper = np.array([117, 255, 255])

        self.yellowLower = np.array([23, 59, 119])
        self.yellowUpper = np.array([54, 255, 255])

        self.orangeLower = np.array([0, 50, 80])
        self.orangeUpper = np.array([20, 255, 255])


    def displayCamera(self, name):
        ret, img = self.cap.read()

        # resize for faster processing
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # mask for the color
        green = cv2.inRange(imgHSV, self.greenLower, self.greenUpper)
        red = cv2.inRange(imgHSV, self.redLower, self.redUpper)
        blue = cv2.inRange(imgHSV, self.blueLower, self.blueUpper)
        yellow = cv2.inRange(imgHSV, self.yellowLower, self.yellowUpper)
        orange = cv2.inRange(imgHSV, self.orangeLower, self.orangeUpper)

        greenOpen = cv2.morphologyEx(green, cv2.MORPH_OPEN, self.kernelOpen)
        greenClose = cv2.morphologyEx(greenOpen, cv2.MORPH_CLOSE, self.kernelClose)

        redOpen = cv2.morphologyEx(red, cv2.MORPH_OPEN, self.kernelOpen)
        redClose = cv2.morphologyEx(redOpen, cv2.MORPH_CLOSE, self.kernelClose)

        blueOpen = cv2.morphologyEx(blue, cv2.MORPH_OPEN, self.kernelOpen)
        blueClose = cv2.morphologyEx(blueOpen, cv2.MORPH_CLOSE, self.kernelClose)

        yellowOpen = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, self.kernelOpen)
        yellowClose = cv2.morphologyEx(yellowOpen, cv2.MORPH_CLOSE, self.kernelClose)

        orangeOpen = cv2.morphologyEx(orange, cv2.MORPH_OPEN, self.kernelOpen)
        orangeClose = cv2.morphologyEx(orangeOpen, cv2.MORPH_CLOSE, self.kernelClose)

        # Track the green color
        _, conts, h = cv2.findContours(greenClose.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(img, conts, -1, (255, 0, 0), 3)

        for i in range(len(conts)):
            x, y, w, h = cv2.boundingRect(conts[i])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "Green", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

        # Track the red color
        _, conts, h = cv2.findContours(redClose.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(img, conts, -1, (255, 0, 0), 3)

        for i in range(len(conts)):
            x, y, w, h = cv2.boundingRect(conts[i])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "Red", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

        # Track the blue color
        _, conts, h = cv2.findContours(blueClose.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(img, conts, -1, (255, 0, 0), 3)

        for i in range(len(conts)):
            x, y, w, h = cv2.boundingRect(conts[i])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "Blue", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

        # Track the yellow color
        _, conts, h = cv2.findContours(yellowClose.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(img, conts, -1, (255, 0, 0), 3)

        for i in range(len(conts)):
            x, y, w, h = cv2.boundingRect(conts[i])
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "Yellow", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

        cv2.imshow(name, img)

