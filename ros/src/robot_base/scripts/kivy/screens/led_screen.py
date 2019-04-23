from kivy.uix.screenmanager import Screen


class LedScreen(Screen):
    def __init__(self, **kw):
        super(LedScreen, self).__init__(**kw)
        # self.led_publisher = rospy.Publisher('led', Led, queue_size=1)

    def send_led_message(self, rgb, section, part, function):
        """
        Publishes a led message to the robot
        :param rgb: red, green and blue list []
        :param section: Legs, Vu, Robot or Face
        :param part: what part of the section (eg. left_eye, right eye)
        :param function: on, off, fade ect.
        :return:
        """
        led_message = Led()
        led_message.rgb = rgb
        led_message.section = section
        led_message.part = part
        led_message.function = function