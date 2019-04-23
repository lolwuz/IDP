#!/usr/bin/env python
import rospy
from adafruit.PCA9685 import *
from robot_controller.msg import MotorState, PwmState


class MotorSubscriber:
    _PWM_PIN = 12
    _SERVO_PIN = 15
    _MOTOR_PINS = {
        'left_front': {"id": 4, "side": "left"},
        'left_mid': {"id": 10, "side": "left"},
        'left_back': {"id": 8, "side": "left"},
        'right_front': {"id": 6, "side": "right"},
        'right_middle': {"id": 0, "side": "right"},
        'right_back': {"id": 2, "side": "right"},
    }
    _FREQUENCY = 50
    _MIN_SERVO_PWM = 120
    _MAX_SERVO_PWM = 246

    def __init__(self):
        """
        The motor subscriber controls the pwm motors
        """
        self.motor_board = PCA9685()

        rospy.Subscriber('motor_state', MotorState, self.motor_state_callback)  # state (left, right, off)
        rospy.Subscriber('motor_pwm', PwmState, self.motor_pwm_callback)  # pwm (speed)
        rospy.Subscriber('motor_servo', PwmState, self.motor_servo_callback)  # Servo (pwm)

        self.motor_board.set_pwm_freq(self._FREQUENCY)  # Communication frequency
        self.motor_board.set_pwm(self._SERVO_PIN, 0, self._MIN_SERVO_PWM)  # Set gripper servo to average
        self.last_message_time = rospy.get_time()

        self.turn_motors_off()

    def motor_pwm_callback(self, data):
        """
        Callback function for setting the pwm (speed) of all the motors
        :param data: robot_controller/PwmState.msg
        :return:
        """
        pwm = abs(data.pwm)

        # Check if pwm is lower than 100
        if pwm < 100:
            self.motor_board.set_pwm(self._PWM_PIN, 0, 0)
        else:
            self.motor_board.set_pwm(self._PWM_PIN, 0, pwm)

        # rospy.loginfo(data.pwm)

    def motor_state_callback(self, data):
        """
        Callback function for setting the direction of a specific motor
        :param data: robot_controller/MotorState.msg
        :return:
        """
        rospy.logout(data)

        pins_to_set = []  # Pins to change state

        if data.name == "all":
            for name in self._MOTOR_PINS:
                pins_to_set.append(self._MOTOR_PINS[name])
        else:
            pins_to_set.append(self._MOTOR_PINS[data.name])

        for motor_data in pins_to_set:
            if motor_data["side"] == "left":
                if data.state == 'up':
                    self.motor_board.set_pwm(motor_data["id"], 0, 4095)
                    self.motor_board.set_pwm(motor_data["id"] + 1, 4095, 0)
                elif data.state == 'down':
                    self.motor_board.set_pwm(motor_data["id"], 4095, 0)
                    self.motor_board.set_pwm(motor_data["id"] + 1, 0, 4095)
                elif data.state == 'off':
                    self.motor_board.set_pwm(motor_data["id"], 0, 0)
                    self.motor_board.set_pwm(motor_data["id"] + 1, 0, 0)
            else:
                if data.state == 'up':
                    self.motor_board.set_pwm(motor_data["id"], 4095, 0)
                    self.motor_board.set_pwm(motor_data["id"] + 1, 0, 4095)
                elif data.state == 'down':
                    self.motor_board.set_pwm(motor_data["id"], 0, 4095)
                    self.motor_board.set_pwm(motor_data["id"] + 1, 4095, 0)
                elif data.state == 'off':
                    self.motor_board.set_pwm(motor_data["id"], 0, 0)
                    self.motor_board.set_pwm(motor_data["id"] + 1, 0, 0)

    def motor_servo_callback(self, data):
        """
        :param data:
        :return:
        """
        if self._MIN_SERVO_PWM > data.pwm or data.pwm < self._MAX_SERVO_PWM:
            self.motor_board.set_pwm(self._SERVO_PIN, 0, data.pwm)
        else:
            rospy.logerr("PWM out of range")

    def turn_motors_off(self):
        # turn all motors off
        for name in self._MOTOR_PINS:
            self.motor_board.set_pwm(self._MOTOR_PINS[name]["id"], 0, 0)
            self.motor_board.set_pwm(self._MOTOR_PINS[name]["id"] + 1, 0, 0)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    motor = MotorSubscriber()
    rospy.spin()
