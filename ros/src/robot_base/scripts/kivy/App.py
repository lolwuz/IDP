#!/usr/bin/env python

import rospy
from kivy.app import App
from kivy.base import runTouchApp
from kivy.lang import Builder
from kivy.properties import StringProperty, ObjectProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.screenmanager import ScreenManager, Screen
from robot_controller.msg import DrivingMode, ClamperMode
# from robot_controller.msg import Led
# from joystick.msg import JoystickState

FACE_EMOTIONS=['']
EYE_EMOTIONS=['Open', 'Square', 'All', 'Sad', 'Closed', 'Off']
MOUTH_EMOTIONS=['Smile', 'Open', 'Neutral', 'Sad', 'Off']

# dropdown = DropDown()
#
# for i in xrange(len(EYE_EMOTIONS)):
#     button = Button(text=EYE_EMOTIONS[i])
#     print(str(dropdown.parent))
#     print(EYE_EMOTIONS[i])
#     dropdown.add_widget(button)
    # self.ids.grid.add_widget(self, button)


# number_of_buttons = 9
#
# menu_buttons = ["Entering
# the arena", "Fast and the furious", "So you can think you can dance", "Line dance",
#                 "Obstacle course", "Cannon", "Transport and rebuild", "Besturing",
#                 "Emoticons"]
#
# function_names = ["module1", "module2", "module3", "module4", "module5", "module6", "module7", "besturing", "emoticons"]
# screen_names = ["Module1Screen", "Module12Screen", "Module3Screen", "Module4Screen", "Module5Screen", "Module6Screen",
#                 "Module7Screen", "Besturing", "Emoticons"]


class MainScreen(Screen):
    #stdoutdata = subprocess.getstatusoutput('hcitool con')

    #if "B8:27:EB:75:44:2C" in stdoutdata.split():
    #    print("we are connected")
    pass


class MenuScreen(Screen):
    pass
    # @mainthread
    # def on_enter(self):
    #     for i in xrange(number_of_buttons):
    #         button = Button(text=menu_buttons[i])
    #         self.ids.grid.add_widget(button)


class Module1Screen(Screen):
    pass


class Module2Screen(Screen):
    pass


class Module3Screen(Screen):
    pass


class Module4Screen(Screen):
    pass


class Module5Screen(Screen):
    pass


class Module6Screen(Screen):
    pass



class Clamper(Screen):
    pass


class Driving(Screen):
    pass


class Emoticons(Screen):
    pass

class LedControl(Screen):
    pass

class LedLegs(Screen):
    dd_legcolor = ObjectProperty(None)
    dd_leganimation = ObjectProperty(None)

class LedFace(Screen):
    dd_leftled = ObjectProperty(None)
    dd_rightled = ObjectProperty(None)
    dd_mouthled = ObjectProperty(None)
    dd_allled = ObjectProperty(None)

class AnotherScreen(Screen):
    pass

class ScreenManagement(ScreenManager):
    pass

presentation = Builder.load_file("main.kv")



class MainApp(App):
    def __init__(self, **kwargs):
        super(MainApp, self).__init__(**kwargs)
        rospy.init_node('kivy_app', anonymous=True)
        self.driving_mode_publisher = rospy.Publisher('driving_mode', DrivingMode, queue_size=1)
        self.clamp_mode_publisher = rospy.Publisher('clamper_mode', ClamperMode, queue_size=1)
        # self.led_publisher = rospy.Publisher('led', Led, queue_size=1)

        # rospy.Subscriber('joystick', JoystickState, self.joystick_callback)

    def sendDrivingMode(self, msg):
        dm_message = DrivingMode()
        dm_message.header.stamp = rospy.Time.now()
        dm_message.mode = msg
        print(str(dm_message))
        self.driving_mode_publisher.publish(dm_message)

    def sendClamperMessage(self, msg):
        c_message = ClamperMode()
        c_message.header.stamp = rospy.Time.now()
        c_message.mode = msg
        print(str(c_message))
        self.clamp_mode_publisher.publish(c_message)

    def testCalculate(self, n1, n2):
        print(n1 + n2)

    def build(self):
        #return CustomDropDown()
        return presentation

    def sendMessage(self, msg):
        led_message = Led()
        led_message.header.stamp = rospy.Time.now()
        led_message.section = msg
        led_message.function = msg

        self.led_publisher.publish(led_message)

    def joystic_callback(self, data):
        rospy.loginfo(data)

if __name__ == '__main__':
    MainApp().run()
