# coding=utf-8
from __future__ import division
import ax12
import math
import sys
from time import sleep

sys.path.insert(0, '../lib')


class Servo:
    def __init__(self):
        self.ax12 = ax12.Ax12()

    def ping(self, servo):
        """
        Ping a given servo
        :param servo: ID of servo
        :return:
        """
        self.ax12.ping(servo)

    def change_id(self, old_servo, new_servo):
        """
        Change the ID of servo
        :param old_servo: old ID of servo
        :param new_servo: new ID of servo
        :return:
        """
        return self.ax12.set_id(old_servo, new_servo)

    def factory_reset(self, servo):
        """
        Resets servo to factory settings
        :param servo: ID of servo
        :return:
        """
        self.ax12.factory_reset(servo, confirm=True)

    def move(self, servo, position):
        """
        Move selected servo to given position
        :param servo: ID of servo
        :param position: goal position (0-1023)
        :return:
        """
        self.ax12.move(servo, int(self.degrees_to_dxl_angle(position)))

    def move_speed(self, servo, position, speed):
        """
        Move selected servo to given position with a selected speed
        :param servo: ID of servo
        :param position: goal position (0-1023)
        :param speed: speed of servo (0-1023)
        :return:
        """
        self.ax12.move_speed(servo, position, speed)
        # self.calculate_delay_between_servos(servo, position, speed)

    def move_speed_rw(self, servo, position, speed):
        """
        Send data buffer to move selected servo to given position with a selected speed and wait with execution
        :param servo: ID of servo
        :param position: goal position (0-1023)
        :param speed: speed of servo (0-1023)
        :return:
        """
        # self.ax12.move_speed_rw(servo, int(self.degrees_to_dxl_angle(position)), speed)
        self.ax12.move_speed_rw(servo, position, speed)

    def action(self):
        """
        Execute all sent data buffers to servo's
        :return:
        """
        self.ax12.action()

    def get_temperature(self, servo):
        """
        Get temperature of servo
        :param servo: ID of servo
        :return:
        """
        self.ax12.read_temperature(servo)

    def get_voltage(self, servo):
        """
        Get voltage of servo
        :param servo: ID of servo
        :return:
        """
        self.ax12.read_voltage(servo)

    def get_position(self, servo):
        """
        Get position of given servo
        :param servo: ID of servo
        :return: current position of servo
        """
        return self.ax12.read_position(servo)

    def get_speed(self, servo):
        """
        Get speed of servo
        :param servo: ID of servo
        :return:
        """
        self.ax12.read_speed(servo)

    def get_load(self, servo):
        """
        Get load of servo
        :param servo: ID of servo
        :return:
        """
        self.ax12.read_load(servo)

    def get_moving_status(self, servo):
        """
        Get moving status of servo
        :param servo: ID of servo
        :return:
        """
        self.ax12.read_moving_status(servo)

    def ping_multiple_servos(self):
        """
        Ping all connected servo's
        :return: list of servo's
        """
        servo_list = []

        y = 0
        for i in range(0, 254):
            # noinspection PyBroadException
            try:
                self.ax12.ping(i)
                sleep(0.002)
                print("Found servo number: {}".format(i))
                servo_list.append(i)
            except Exception:
                y = y + 1

        return servo_list

    @staticmethod
    def degrees_to_dxl_angle(angle_degrees):
        """Normalize the given angle.

        PxAX-12 uses the position angle (-150.0°, +150.0°) range instead of the
        (0°, +300.0°) range defined in the Dynamixel official documentation because
        the former is easier to use (especially to make remarkable angles like
        right angles or 45° and 135° angles).

        :param float angle_degrees: an angle defined in degrees the range
            (-150.0°, +150.0°) where:

            - -150.0 is a 150° clockwise angle;
            - +150.0 is a 150° counter clockwise angle.

        :return: an angle defined according to the Dynamixel internal notation,
            i.e. in the range (0, 1023) where:

            - 0 is a 150° clockwise angle;
            - 1023 is a 150° counter clockwise angle.

        :rtype: int.
        """
        dxl_angle = math.floor((angle_degrees + 150.0) / 300. * 1023.)
        return dxl_angle

    @staticmethod
    def dxl_angle_to_degrees(dxl_angle):
        """Normalize the given angle.

        PxAX-12 uses the position angle (-150.0°, +150.0°) range instead of the
        (0°, +300.0°) range defined in the Dynamixel official documentation because
        the former is easier to use (especially to make remarkable angles like
        right angles or 45° and 135° angles).

        :param int dxl_angle: an angle defined according to the Dynamixel internal
            notation, i.e. in the range (0, 1023) where:

            - 0 is a 150° clockwise angle;
            - 1023 is a 150° counter clockwise angle.

        :return: an angle defined in degrees in the range (-150.0°, +150.0°) where:

            - -150.0 is a 150° clockwise angle;
            - +150.0 is a 150° counter clockwise angle.

        :rtype: float.
        """
        angle_degrees = round(dxl_angle / 1023. * 300. - 150.0, 1)
        return angle_degrees

    def calculate_delay_between_servos(self, id, new_angle, speed, verbose=False):
        """
        Calculates the delay until servo is done with movement
        :param verbose: boolean to print data to terminal
        :param id: of servo to move
        :param new_angle: position to go to
        :param speed: speed desired
        :return: delay time
        """
        angle_per_second = 114 * 360 / 60
        prev = self.dxl_angle_to_degrees(self.get_position(id))
        delta = abs(self.dxl_angle_to_degrees(new_angle) - prev)
        t = float(delta / (angle_per_second * (speed / 1023)))
        if verbose:
            print("Servo: \t" + str(id) + " \tAngle: \t" + str(self.dxl_angle_to_degrees(new_angle))
                  + " \tPrevious angle: \t"
                  + str(prev) + " \tDelta: \t" + str(delta) + " \tTime to sleep: \t" + str(t))
        sleep(t)

    # deprecated
    def write_move_reg(self, indexes, positions):
        for i in range(0, len(indexes)):
            self.ax12.write_move_reg(indexes[i], (positions[i] % 256, positions[i] >> 8))
            print("-----------")
        self.ax12.action_reg(0xFE)

    # deprecated
    def write_move_speed_reg(self, indexes, positions, speeds):
        for i in range(0, len(indexes)):
            print(indexes[i], positions[i], speeds[i])
            self.ax12.write_move_speed_reg(indexes[i], (positions[i] % 256, positions[i] >> 8),
                                           (speeds[i] % 256, speeds[i] >> 8))
            sleep(0.00002)
        self.ax12.action_reg(0xFE)

    def sync_write_move_speed(self, id_array, position_array, speed):
        """
        Sync write to move multiple servo at the same time
        :param id_array: array containing id's of all servos to move
        :param position_array: positions to move to relative to the id_array
        :param speed: speed with which to move the servos
        :return: None
        """
        self.ax12.sync_write_move_speed(id_array, position_array, speed)
