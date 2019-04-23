import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while (1):
    # Take each frame
    _, frame = cap.read()

    gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    canny = cv2.Canny(blurred, 20, 40)

    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(canny, kernel, iterations=2)

    _, contours, hierarchy = cv2.findContours(dilated.copy(), 1, 2)


    for cnt in contours:
        pass
    # Show keypoints

    cv2.imshow("blurred", blurred)
    cv2.imshow("dilated", dilated)
    cv2.imshow("squares", frame)


    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
