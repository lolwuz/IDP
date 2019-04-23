#!/usr/bin/env python
import rospy
import math
from robot_controller.msg import MotorState, PwmState, SteerState, GoalPosition, Led, GripperState
from robot_base.msg import BaseMsg
from sensor_msgs.msg import JointState


class BaseNode(object):

    def __init__(self, name, is_running, rate):
        """ Base node with default functions """
        rospy.init_node('robot_base_' + name, anonymous=True)

        # Publishers for controller, motor direction, pwm, servo_steer and servo_suspension
        self.motor_state_publisher = rospy.Publisher('motor_state', MotorState, queue_size=1)
        self.motor_pwm_publisher = rospy.Publisher('motor_pwm', PwmState, queue_size=1)
        self.motor_servo_publisher = rospy.Publisher('motor_servo', PwmState, queue_size=1)
        self.servo_steer_publisher = rospy.Publisher('servo_steer', JointState, queue_size=1)
        self.servo_suspension_publisher = rospy.Publisher('suspension', JointState, queue_size=1)
        self.goal_position_publisher = rospy.Publisher('set_goal_position', GoalPosition, queue_size=1)
        self.led_publisher = rospy.Publisher('led', Led, queue_size=1)
        self.set_state_gripper = rospy.Publisher('change_gripper_state', GripperState, queue_size=1)

        # Base subscriber
        rospy.Subscriber('robot_base', BaseMsg, self.base_callback)

        self.start_time = 0
        self.move_array = []  # Array of moves that are already executed
        self.led_array = []
        self.motor_array = []

        # Update loop
        self.rate = rospy.Rate(rate)

        self.is_running = is_running
        self.name = name

    def base_callback(self, data):
        """
        Base callback from remote
        :param data:
        :return:
        """
        rospy.logout(data)
        if data.base_msg == self.name:
            self.is_running = True
            self.move_array = []
            self.led_array = []
            self.motor_array = []
            self.start_time = rospy.get_time()
        else:
            self.is_running = False

    def update(self):
        """ Default update loop with sleep """
        self.rate.sleep()

    def set_goal_position(self, id_array, position_array, speed_array):
        """
        Set servo goal position from dxl angle
        :return:
        """
        goal_message = GoalPosition()
        goal_message.header.stamp = rospy.Time.now()

        goal_message.id_array = id_array
        goal_message.position_array = position_array
        goal_message.speed_array = speed_array

        self.goal_position_publisher.publish(goal_message)

    def change_motor_state(self, name, direction):
        """
        Send the command to change the direction of a motor
        :param name: name of motor to change. "all" for all motors
        :param direction: "left", "right" or "off"
        """
        motor_message = MotorState()
        motor_message.header.stamp = rospy.Time.now()
        motor_message.name = name
        motor_message.state = direction

        self.motor_state_publisher.publish(motor_message)

    def change_pwm_state(self, pwm):
        """
        Send the command to change the pwm speed of all the motors.
        :param pwm: pwm value in the range 0-4095
        :return:
        """
        pwm_message = PwmState()
        pwm_message.header.stamp = rospy.Time.now()
        pwm_message.pwm = pwm

        self.motor_pwm_publisher.publish(pwm_message)

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

    def change_gripper_state(self, clamp=True):
        """
        Changes the gripper state from clamp to unclamp
        :return:
        """
        gripper_state = GripperState()
        gripper_state.header.stamp = rospy.Time.now()
        gripper_state.clamp = clamp

        self.set_state_gripper.publish(gripper_state)



