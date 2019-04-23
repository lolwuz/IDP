from Vision import Video
import cv2

if '__main__' == __name__:

    vid = Video()


    while (True):
        vid.displayCamera("Main")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



