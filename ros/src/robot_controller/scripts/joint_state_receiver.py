#!/usr/bin/env python
from __future__ import division

import math
import rospy
import smbus
from robot_controller.msg import GoalPosition, GripperState
from sensor_msgs.msg import JointState

from lib.servo import Servo
from lib.servo_dict import servo_dict


class JointSubscriber:

    def __init__(self):
        """
        The joint subscriber controls the AX-12A servo's
        """
        self.servo = Servo()
        rospy.Subscriber('servo_steer', JointState, self.steer_state_callback)  # Steering
        rospy.Subscriber('suspension', JointState, self.suspension_state_callback)  # Suspension
        rospy.Subscriber('set_goal_position', GoalPosition, self.set_goal_position)  # Dance goal position
        rospy.Subscriber('change_gripper_state', GripperState, self.change_state_gripper)  # Gripper goal position

        # self.joint_state_publisher = rospy.Publisher('', JointState, queue_size=1)
        # self.goal_position_service = rospy.Service('goal_position', GoalPosition, self.handle_goal_position)

        self.servos = [20, 21]

        self.bus = smbus.SMBus(1)
        self.bus.write_byte(0x68, 0x10)

        self.set_servo_start()  # Center all servo's on start
        # self.unclamp_both_servos()

    def steer_state_callback(self, data):
        """
        Handles the steering topic
        :param data: sensor_msgs/JointState
        :return:
        """
        id_array = []
        position_array = []
        velocity_array = []

        for i in range(len(data.name)):
            joint_name = data.name[i]
            joint_speed = data.velocity[i]
            servo = servo_dict[joint_name]

            servo_id = servo["id"]
            servo_max = servo["max"]
            servo_min = servo["min"]

            difference = abs(servo_max - servo_min)

            if data.position[i] > 180:
                dxl_difference = difference * ((data.position[i] - 180) / 180)
            else:
                dxl_difference = difference * (data.position[i] / 180)

            if servo_min < servo_max:
                dxl_angle = servo_min + dxl_difference
            else:
                dxl_angle = servo_max + dxl_difference

            id_array.append(int(servo_id))
            position_array.append(int(dxl_angle))
            velocity_array.append(int(joint_speed))

        self.set_servo_state(id_array, position_array, velocity_array)

    def suspension_state_callback(self, data):
        """
        Handles the suspension (up/dowm) movement of the robot while in driving mode
        :param data:
        :return:
        """
        id_array = []
        position_array = []
        velocity_array = []

        for i in range(len(data.name)):
            joint_name = data.name[i]
            joint_speed = data.velocity[i]
            servo = servo_dict[joint_name]

            servo_id = servo["id"]
            servo_max = servo["max"]
            servo_min = servo["min"]
            servo_mid = (servo_min + servo_max) / 2

            difference = abs(servo_max - servo_min)
            dxl_difference = (difference * data.position[i]) / 2

            if servo_id == 5 or servo_id == 4:
                if servo_min < servo_max:
                    dxl_angle = servo_mid + dxl_difference
                else:
                    dxl_angle = servo_mid + dxl_difference
            else:
                if servo_min < servo_max:
                    dxl_angle = servo_mid - dxl_difference
                else:
                    dxl_angle = servo_mid - dxl_difference

            id_array.append(int(servo_id))
            position_array.append(int(dxl_angle))
            velocity_array.append(int(joint_speed))

        self.set_servo_state(id_array, position_array, velocity_array)

    def set_servo_start(self):
        """
        Returns all servo's to their starting position (center)
        :return:
        """
        id_array = []
        position_array = []
        speed_array = []

        for name in servo_dict:
            servo = servo_dict[name]

            servo_id = servo["id"]
            servo_min = servo["min"]
            servo_max = servo["max"]
            servo_avg = int((servo_max + servo_min) / 2)
            servo_function = servo["def"]

            id_array.append(servo_id)

            if servo_function == "steering":
                position_array.append(servo_avg)
            elif servo_function == "suspension":
                position_array.append(servo_min)

            speed_array.append(30)

        self.servo.sync_write_move_speed(id_array, position_array, speed_array)

    def set_servo_state(self, id_array, position_array, speed):
        """
        Checks if supplied positions are within range to prevent ax12 servo overload
        :param id_array:
        :param position_array:
        :param speed:
        :return:
        """
        for name in servo_dict:
            servo = servo_dict[name]

            for i in range(len(id_array)):
                if servo["id"] == id_array[i]:
                    servo_max = servo["max"]
                    servo_min = servo["min"]

                    if servo_min < servo_max:
                        range_servo = range(servo_min + 1, servo_max - 1)
                    else:
                        range_servo = range(servo_max + 1, servo_min - 1)

                    if position_array[i] not in range_servo:
                        rospy.loginfo(servo["id"])
                        rospy.loginfo(position_array[i])
                        return

        self.servo.sync_write_move_speed(id_array, position_array, speed)

    # ---------------------------------------------------  TEST  ------------------------------------------------------#
    def set_goal_position(self, data):
        """
        Gets a goal position from the base_node. Position array of angle (degrees)
        :param data: robot_controller/GoalPosition
        :return:
        """
        id_array = []
        dxl_position_array = []
        speed_array = []

        for name in servo_dict:
            servo_obj = servo_dict[name]
            servo_id = servo_obj["id"]
            servo_function = servo_obj["def"]
            servo_min = self.servo.dxl_angle_to_degrees(servo_obj["min"])
            servo_max = self.servo.dxl_angle_to_degrees(servo_obj["max"])
            difference = abs(servo_min - servo_max)

            for i in range(len(data.id_array)):
                if data.id_array[i] == servo_id:
                    if servo_function == "suspension":
                        factor = self.get_factor(name)
                        percentage = data.position_array[i] / 40.0
                        angle_addition = percentage * difference
                        servo_angle = servo_min + angle_addition * factor
                        dxl_angle = self.servo.degrees_to_dxl_angle(servo_angle)
                    elif servo_function == "steering":
                        if servo_min <= servo_max:
                            factor = 1
                        else:
                            factor = -1
                        percentage = data.position_array[i] / 180.0
                        angle_addition = percentage * difference
                        servo_angle = servo_min + angle_addition * factor
                        dxl_angle = self.servo.degrees_to_dxl_angle(servo_angle)
                    else:
                        rospy.logerr("No function for servo joints")
                        return

                    id_array.append(servo_id)
                    dxl_position_array.append(int(dxl_angle))
                    speed_array.append(data.speed_array[i])

        rospy.loginfo(id_array)
        rospy.loginfo(dxl_position_array)
        rospy.loginfo(speed_array)

        self.servo.sync_write_move_speed(id_array, dxl_position_array, speed_array)

    def get_factor(self, name):
        factor = 0
        if name.endswith("a"):
            if name.split("_")[2].split("a")[0] == 'l':
                factor = 1
            elif name.split("_")[2].split("a")[0] == 'r':
                factor = -1
        if name.endswith("b"):
            if name.split("_")[2].split("b")[0] == 'l':
                factor = 1
            elif name.split("_")[2].split("b")[0] == 'r':
                factor = -1
        if name.endswith("c"):
            if name.split("_")[2].split("c")[0] == 'l':
                factor = -1
            elif name.split("_")[2].split("c")[0] == 'r':
                factor = 1

        return factor

    def publish_joint_state(self):

        joint_state_message = JointState()
        joint_state_message.header.stamp = rospy.Time.now()

        radian_range = math.radians(300)

        for name in servo_dict:
            servo = servo_dict[name]

            servo_min = servo["min"]
            servo_max = servo["max"]

            min_radians = math.radians(self.servo.dxl_angle_to_degrees(servo_min))
            max_radians = math.radians(self.servo.dxl_angle_to_degrees(servo_max))

            dxl_position = self.servo.get_position(servo["id"])

            position_degrees = self.servo.dxl_angle_to_degrees(dxl_position)
            position_radians = math.radians(position_degrees)

            radian_translation = min_radians + position_radians + max_radians

            joint_state_message.position.append()

    def unclamp_both_servos(self):
        self.clamp_outwards()
        self.clamp_up()

    def change_state_gripper(self, data):
        if data.clamp:
            self.clamp_down()
            self.clamp_inwards()

        else:
            self.clamp_up()
            self.clamp_outwards()

    def clamp_inwards(self):
        self.servo.ax12.move_speed(self.servos[1], 700, 800)

        while True:
            data = self.bus.read_i2c_block_data(0x68, 0x00, 2)
            # Convert the data to 12-bits
            rospy.sleep(0.5)
            raw_adc = (data[0] & 0x0F) * 256 + data[1]
            print(data)
            if raw_adc > 86:
                self.servo.move_speed(self.servos[1], 100, 0)
                print("Clamped!")
                break
            print "Digital value of Analog Input : %f" % raw_adc

    def clamp_down(self):
        self.servo.ax12.move_speed(self.servos[0], 700, 1800)

        while True:
            data = self.bus.read_i2c_block_data(0x68, 0x00, 2)
            # Convert the data to 12-bits
            rospy.sleep(0.5)
            raw_adc = (data[0] & 0x0F) * 256 + data[1]
            print(data)
            if raw_adc > 80:
                self.servo.move_speed(self.servos[0], 100, 0)
                print("Clamped!")
                break
            print "Digital value of Analog Input : %f" % raw_adc

    def clamp_up(self):
        self.servo.ax12.move_speed(self.servos[0], 700, 800)
        rospy.sleep(6)
        self.servo.ax12.move_speed(self.servos[0], 700, 0)

    def clamp_outwards(self):
        self.servo.ax12.move_speed(self.servos[1], 700, 1800)
        rospy.sleep(6)
        self.servo.ax12.move_speed(self.servos[1], 700, 0)


if __name__ == '__main__':
    rospy.init_node('joint_state_receiver', anonymous=True)
    motor = JointSubscriber()
    rospy.spin()
