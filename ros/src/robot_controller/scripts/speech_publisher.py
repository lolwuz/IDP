import pyttsx
import time

import rospy
from src.robot_base.scripts.base.base_node import BaseNode
from motor_controller.msg import SpeechState


class SpeechPublisher(BaseNode):

    def __init__(self):
        self.engine = pyttsx.init()

        self.currently_speaking_publisher = rospy.Publisher('currently_speaking', self.engine.isBusy(), queue_size=1)


        while not rospy.is_shutdown():
            self.update()

    def update(self):

        speech_message = SpeechState()
        speech_message.header.stamp = rospy.Time.now()
        speech_message.isbusy = self.engine.isBusy()

        self.currently_speaking_publisher.publish(speech_message)


if __name__ == '__main__':
    try:
        speech_pub = SpeechPublisher()
    except rospy.ROSInterruptException:
        pass