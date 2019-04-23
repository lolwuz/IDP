#!/usr/bin/env python
import rospy
from robot_controller.msg import Led, AudioState
from leds.Led import Led as LedController


class LedSubscriber:
    number_of_leds = 56
    brightness = 255

    def __init__(self):
        """
        The led subscriber controls the AdaFruit led's
        """
        rospy.Subscriber('led', Led, self.led_callback)
        rospy.Subscriber('audio', AudioState, self.audio_callback)

        self.ledcontroller = LedController(self.number_of_leds, self.brightness)
        self.update_rate = rospy.Rate(24)

        self.rgb = [0, 0, 0]
        self.section = ''
        self.part = ''
        self.function = ''

        while not rospy.is_shutdown():
            self.update()

    def led_callback(self, data):
        self.section = data.section
        self.part = data.part
        self.rgb = data.rgb
        self.function = data.function

    def audio_callback(self, data):
        band_strength = data.band_strength

        self.ledcontroller.vu.lowband.animate(band_strength[0])
        self.ledcontroller.vu.midband.animate(band_strength[1])
        self.ledcontroller.vu.highband.animate(band_strength[2])

    def update(self):

        if self.section == "robot":
            func = getattr(self.ledcontroller, self.function)
            func(self.rgb[0], self.rgb[1], self.rgb[2])
        elif self.section == "vu":
            func = getattr(self.ledcontroller.vu, self.function)
            # func(data.)

        elif self.section == "face":
            if self.part == "mouth":
                func = getattr(self.ledcontroller.face.mouth, self.function)
                func(self.rgb[0], self.rgb[1], self.rgb[2])
            elif self.part == "right_eye":
                func = getattr(self.ledcontroller.face.righteye, self.function)
                func(self.rgb[0], self.rgb[1], self.rgb[2])
            elif self.part == "left_eye":
                func = getattr(self.ledcontroller.face.lefteye, self.function)
                func(self.rgb[0], self.rgb[1], self.rgb[2])
        elif self.section == "legs":
            func = getattr(self.ledcontroller.legs, self.function)
            func(self.rgb[0], self.rgb[1], self.rgb[2])

        self.update_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('led_subscriber', anonymous=True)
    leds = LedSubscriber()
    rospy.spin()
