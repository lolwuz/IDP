#!/usr/bin/env python
import math
import time

import rospy
import pygame
from robot_controller.msg import Led, PwmState, GoalPosition
from robot_base.msg import BaseMsg
# Kivy dependencies
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen, NoTransition
from screens import *
from kivy.config import Config


class MainApp(App):
    def __init__(self, **kwargs):
        """ The mainApp object """
        super(MainApp, self).__init__(**kwargs)

        pygame.init()
        pygame.mixer.music.load('/home/pi/ros/src/robot_base/scripts/kivy/music.mp3')

        rospy.init_node('app_node', anonymous=True)
        self.base_publisher = rospy.Publisher('robot_base', BaseMsg, queue_size=1)
        self.led_publisher = rospy.Publisher('led', Led, queue_size=1)
        self.motor_servo_publisher = rospy.Publisher('motor_servo', PwmState, queue_size=1)
        self.goal_position_publisher = rospy.Publisher('set_goal_position', GoalPosition, queue_size=1)

    def build(self):
        """ Initialize screen_manager and add screens """
        Config.set('graphics', 'width', '480')
        Config.set('graphics', 'height', '320')

        sm = ScreenManager()
        sm.add_widget(picker_screen.PickerScreen(name="picker_screen"))
        sm.add_widget(led_screen.LedScreen(name='led_screen'))
        sm.add_widget(servo_screen.ServoScreen(name='servo_screen'))
        sm.add_widget(setting_screen.SettingScreen(name='setting_screen'))
        sm.add_widget(mode_screen.ModeScreen(name='mode_screen'))
        sm.transition = NoTransition()
        return sm

    def send_base_msg(self, msg, args):
        """
        Publish a base message
        :param msg: base name
        :param args: arguments list [] for the base
        """
        base_msg = BaseMsg()
        base_msg.header.stamp = rospy.Time.now()

        base_msg.base_msg = msg
        base_msg.args = args

        if msg == 'dancing':
            print('dancing!')
            pygame.mixer.music.play()
            rospy.sleep(0.24)

        self.base_publisher.publish(base_msg)

    def set_led_function(self, rgb, section, part, function):
        """
        :param section: led section of the robot
        :param function: function off led section
        :return:
        """
        led_message = Led()
        led_message.header.stamp = rospy.Time.now()
        led_message.section = section
        led_message.part = part
        led_message.function = function
        led_message.rgb = rgb
        self.led_publisher.publish(led_message)

    def set_gripper_state(self, pwm):
        """
        Set the gripper state (pwm) in range (246, 491) open/close
        :param pwm: pwm value to set
        :return:
        """
        servo_message = PwmState()
        servo_message.header.stamp = rospy.Time.now()
        servo_message.pwm = pwm

        self.motor_servo_publisher.publish(servo_message)

    def set_goal_position(self, left, right, steer):
        left = int(round(left))
        right = int(round(right))
        steer = int(round(steer))

        id_array = [9, 7, 5, 8, 6, 4, 22, 16, 17, 10, 19, 18]
        position_array = [left, left, left, right, right, right, steer, steer, steer, steer, steer, steer]
        speed_array = []

        for i in range(0, 13):
            speed_array.append(50)

        goal_message = GoalPosition()
        goal_message.header.stamp = rospy.Time.now()

        goal_message.id_array = id_array
        goal_message.position_array = position_array
        goal_message.speed_array = speed_array

        print(goal_message)

        self.goal_position_publisher.publish(goal_message)


if __name__ == '__main__':
    MainApp().run()
