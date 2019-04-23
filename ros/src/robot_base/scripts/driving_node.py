#!/usr/bin/env python
import rospy
import math
from base.base_node import BaseNode
from joystick.msg import JoystickState
from robot_controller.msg import MotorState, PwmState, SteerState
from sensor_msgs.msg import JointState
from sensor_controller.msg import GyroState


class Joy:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.button = False


class Gyroscope:
    def __init__(self):
        self.acceleration = [0, 0, 0]
        self.rotation = [0, 0, 0]


class DrivingNode(BaseNode):

    _WHEEL_BASE = 125
    _WHEEL_WIDTH = 140
    _MAX_ANGLE = 60
    _MAX_STEER_VELOCITY = 200
    _MAX_SUSPENSION_ANGLE = 47

    _DRIVING_JOINTS = {
        "steering_joint_la": "left",
        "steering_joint_ra": "right",
        "steering_joint_lb": "left",
        "steering_joint_rb": "right",
        "steering_joint_lc": "left",
        "steering_joint_rc": "right"
    }

    _SUSPENSION_JOINTS = {
        "suspension_joint_la": "left",
        "suspension_joint_ra": "right",
        "suspension_joint_lb": "left",
        "suspension_joint_rb": "right",
        "suspension_joint_lc": "left",
        "suspension_joint_rc": "right"
    }

    def __init__(self):
        """ Node for controlling the driving robot """
        super(DrivingNode, self).__init__("driving", True, 24)
        rospy.Subscriber('joystick', JoystickState, self.joystick_callback)  # Joystick subscriber
        rospy.Subscriber('gyroscope_robot', GyroState, self.gyro_callback)  # Gyroscope subscriber

        # Controls
        self.pwm = 0
        self.current_direction = "up"
        self.left_joy = Joy()  # Joy1, speed
        self.right_joy = Joy()  # Joy2, direction and speed
        self.gyroscope = Gyroscope()
        self.last_message_time = rospy.get_time()

        while not rospy.is_shutdown() and self.is_running:
            self.update()

    def gyro_callback(self, data):
        """
        Gyro_callback handler
        :param data: sensor_controller/GiroState
        """
        self.gyroscope = data

    def joystick_callback(self, data):
        """
        Joystick_callback handler
        :param data: joystick/JoystickState.msg
        """
        self.last_message_time = rospy.get_time()

        self.left_joy = data.joys[0]  # Left joy
        self.right_joy = data.joys[1]  # Right joy

    def update(self):
        """
        Function that decides what to update
        :return:
        """
        super(DrivingNode, self).update()
        self.update_motor()
        self.update_steer()
        self.update_suspension()
        self.update_gripper()
        self.update_led()

    def update_motor(self):
        """
        Publishes the new motor state
        :return:
        """
        # Check for controller timeout
        current_time = rospy.get_time()
        time_difference = current_time - self.last_message_time

        if time_difference > 0.6:
            self.change_motor_state("all", "off")
            return

        # Calculate new_pwm
        speed = 1012 - self.left_joy.y
        new_pwm = speed * 4

        # TURBO ?
        if new_pwm > 3500:
            new_pwm = 4094

        # Check for a change in direction
        if new_pwm < -300:
            if self.pwm > -300:
                self.change_motor_state("all", "down")
        elif new_pwm > 300:
            if self.pwm < 300:
                self.change_motor_state("all", "up")
        else:
            self.change_motor_state("all", "off")

        # Set new PWM to current pwm and send
        pwm_difference = abs(self.pwm - new_pwm)

        if self.pwm > new_pwm:
            self.pwm = self.pwm - pwm_difference / 4
        else:
            self.pwm = self.pwm + pwm_difference / 4

        self.change_pwm_state(self.pwm)

    def update_steer(self):
        """
        Publishes the new steering
        :return:
        """
        percentage = (self.right_joy.x - 1000.) / 1000.

        # rospy.logout(percentage)

        # The minimal steering angle is 45 percent
        steer_angle_degrees = abs(self._MAX_ANGLE * percentage)
        steer_angle_radian = math.radians(steer_angle_degrees)

        radius_a = 0  # Inner wheel
        radius_b = 0  # Outer wheel

        multiplier = 0  # Multiplier 1 for left -1 for right
        if percentage > 0:
            radius_a = self._WHEEL_BASE / math.sin(steer_angle_radian)
            radius_b = radius_a + self._WHEEL_WIDTH * 2
            multiplier = 1
        elif percentage < 0:
            radius_b = self._WHEEL_BASE / math.sin(steer_angle_radian)
            radius_a = radius_b + self._WHEEL_WIDTH * 2
            multiplier = -1

        if radius_a == 0 or radius_b == 0:
            angle_right = 0
            angle_left = 0
        else:
            angle_right = math.asin(self._WHEEL_BASE / radius_a) * multiplier
            angle_left = math.asin(self._WHEEL_BASE / radius_b) * multiplier

        # Don't steer on minimal joystick.
        if abs(percentage) < 0.1:
            angle_right = 0
            angle_left = 0

        # Joint message
        joint_message = JointState()
        joint_message.header.stamp = rospy.Time.now()

        for name in self._DRIVING_JOINTS:
            joint_message.name.append(name)
            joint_message.velocity.append(200)
            joint_message.effort.append(0)

            if name == "steering_joint_la":
                joint_message.position.append(270 + math.degrees(angle_left))
            if name == "steering_joint_ra":
                joint_message.position.append(90 + math.degrees(angle_right))
            if name == "steering_joint_lb":
                joint_message.position.append(270)
            if name == "steering_joint_rb":
                joint_message.position.append(90)
            if name == "steering_joint_lc":
                joint_message.position.append(270 + math.degrees(-angle_left))
            if name == "steering_joint_rc":
                joint_message.position.append(90 + math.degrees(-angle_right))

        # Publish joint_state
        self.servo_steer_publisher.publish(joint_message)

    def update_steer_motor(self, c):
        """
        Free wheel mode
        :param c: value of how far the joystick is pushed
        :return:
        """
        x = self.right_joy.x - 1000
        y = self.right_joy.y - 1000

        angle_radians = math.atan2(y, x)
        angle_degrees = math.degrees(angle_radians) + 90
        angle_degrees_opposite = (angle_degrees + 180) % 360

        if x < 0 and y < 0:
            angle_degrees = angle_degrees + 360

        # Joint message
        joint_message = JointState()
        joint_message.header.stamp = rospy.Time.now()

        for key in self._DRIVING_JOINTS:
            joint_message.name.append(key)
            joint_message.velocity.append(self._MAX_STEER_VELOCITY)
            joint_message.effort.append(0)

            if self._DRIVING_JOINTS[key] == "right":
                if angle_degrees <= 180:
                    joint_message.position.append(angle_degrees)
                else:
                    joint_message.position.append(angle_degrees_opposite)

            if self._DRIVING_JOINTS[key] == "left":
                if angle_degrees <= 180:
                    joint_message.position.append(angle_degrees_opposite)
                else:
                    joint_message.position.append(angle_degrees)

        self.servo_steer_publisher.publish(joint_message)

        # Change the motor direction
        if angle_degrees <= 180:
            if self.current_direction != "left":
                self.change_motor_state("all", "up")
        else:
            if self.current_direction != "right":
                self.change_motor_state("all", "down")

        # Change the pwm speed
        self.pwm = c
        self.change_pwm_state(self.pwm)

    def update_suspension(self):
        """ Update suspension servo's """
        percentage = (self.right_joy.x - 1000.) / 1000.

        degrees_to_turn = percentage * self._MAX_SUSPENSION_ANGLE / 2
        suspension_degrees = self._MAX_SUSPENSION_ANGLE / 2 + degrees_to_turn

        # Joint message
        joint_message = JointState()
        joint_message.header.stamp = rospy.Time.now()

        for key in self._SUSPENSION_JOINTS:
            joint_message.name.append(key)
            joint_message.velocity.append(50)
            joint_message.position.append(percentage)

            joint_message.effort.append(0)

        self.servo_suspension_publisher.publish(joint_message)

    def update_gripper(self):
        """ Updates the gripper position with the joystick button """
        if self.right_joy.button and not self.left_joy.button:
            self.set_gripper_state(240)

        if self.left_joy.button and not self.right_joy.button:
            self.set_gripper_state(130)

    def update_led(self):
        """ Update the leds when driving """
        if self.pwm < 300:
            self.set_led_function([255, 0, 0], "legs", "", "")
        else:
            percentage = self.pwm / 4095
            blue = 255 * percentage
            self.set_led_function([0, 0, blue], "legs", "", "all")

if __name__ == '__main__':
    try:
        driving_node = DrivingNode()
    except rospy.ROSInterruptException:
        pass
