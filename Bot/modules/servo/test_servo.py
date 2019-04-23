from __future__ import division

import logging
from random import randint
from time import sleep

from Servo import Servo

logging.basicConfig()

servo = Servo()


def infinity():
    prev_1 = 10
    angle_per_sec = 114 * 360 / 60
    speed = 100
    while True:
        # noinspection PyBroadException
        try:
            ang = randint(-45, 45)
            delta = abs(ang - prev_1)
            t = float(delta / (angle_per_sec * (speed / 1023)))  # 40 / (684 * (100 / 1023))
            print("Angle: ", ang, "previous angle: ", prev_1, "delta:", delta, "Time to sleep: ", t)
            servo.move_speed(1, ang, speed)
            sleep(t)
            servo.move_speed(2, ang, speed)
            sleep(t)
            servo.move_speed(3, ang, speed)
            sleep(t)
            prev_1 = ang
        except Exception:
            logging.error('Failed.', exc_info=True)
            break


# infinity()


def test_implementation_delay():
    servo.move_speed(1, 520, 100)
    servo.move_speed(2, 520, 100)
    servo.move_speed(3, 520, 100)
    servo.move_speed(1, 350, 100)
    servo.move_speed(2, 350, 100)
    servo.move_speed(3, 350, 100)
    servo.move_speed(1, 700, 100)
    servo.move_speed(2, 700, 100)
    servo.move_speed(3, 700, 100)
    servo.move_speed(1, 520, 100)
    servo.move_speed(2, 520, 100)
    servo.move_speed(3, 520, 100)
    servo.move_speed(1, 350, 100)
    servo.move_speed(2, 350, 100)
    servo.move_speed(3, 350, 100)
    servo.move_speed(1, 700, 100)
    servo.move_speed(2, 700, 100)
    servo.move_speed(3, 700, 100)


def test_rw_700():
    servo.move_speed_rw(1, 500, 500)
    servo.move_speed_rw(2, 500, 500)
    servo.move_speed_rw(3, 500, 500)
    servo.action()


def sway():
    # right left
    servo.ax12.sync_write_move_speed([17, 16, 18, 19, 10],
                                     [520, 520, 820, 520, 820], 200)

    sleep(2)
    # reverse
    servo.ax12.sync_write_move_speed([17, 16, 18, 19, 10],
                                     [920, 920, 320, 50, 350], 200)
    sleep(2)


def center_all():
    # right left
    servo.ax12.sync_write_move_speed([17, 16, 18, 19, 10, 5, 7, 9, 4, 6, 8],
                                     [659, 674, 670, 360, 666, 654, 364, 367, 361, 652, 660], 200)


def face_up():
    servo.ax12.sync_write_move_speed([5, 7, 9, 4, 6, 8],
                                     [586, 428, 431, 432, 581, 590], 200)


def face_down():
    servo.ax12.sync_write_move_speed([5, 7, 9, 4, 6, 8],
                                     [722, 301, 303, 290, 724, 730], 200)


def steer():
    servo.ax12.sync_write_move_speed([17, 16, 18, 19, 10],
                                     [350, 380, 973, 668, 963], 200)
    sleep(2)
    servo.ax12.sync_write_move_speed([17, 16, 18, 19, 10],
                                     [969, 969, 367, 52, 369], 200)


# speed = 100
# while True:
#     position = 520
#     servo.ax12.sync_write_move_speed([1, 2, 3, 15, 14, 13, 11],
#                                      [position, position, position, position, position, position, position], speed)
#     sleep(2)
#     position = 320
#     servo.ax12.sync_write_move_speed([1, 2, 3, 15, 14, 13, 11],
#                                      [position, position, position, position, position, position, position], speed)
#
#     sleep(2)
#     position = 720
#     servo.ax12.sync_write_move_speed([1, 2, 3, 15, 14, 13, 11],
#                                      [position, position, position, position, position, position, position], speed)
#     sleep(3)


# import serial
# import time
# from threading import Lock
#
# mutex = Lock()
#
# mutex.acquire()
# try:
#     sendSerial.write ("data1" + "data2" + "data3" + "data4")
# except:
#     pass
# finally:
#     self.mutex.release()
#
# while True:
#     with mutex:
#         readoutSerial.read ("data5" + "data6")


# servo.move_speed(1, 520, 100)
# servo.move_speed(1, 320, 100)
# servo.move_speed(1, 720, 100)


# servo.ping_multiple_servos()

# servo.move_speed(19, 50, 200)
# print(servo.get_position(19))
# servo.move_speed(3, 1020, 1023)

# position = 520
# servo.ax12.sync_write_move_speed([4, 5, 6, 7],
#                                  [309, 592, 584, 309], 200)
# servo.move_speed(4, 309, 200)
# servo.move_speed(20, 1023, 2047)
# servo.move_speed(17, 520, 200)
# servo.move_speed(16, 520, 200)
# servo.move_speed(18, 820, 200)
# servo.move_speed(19, 520, 200)
# servo.move_speed(10, 820, 200)
# print(servo.get_position(20))
# servo.ping_multiple_servos()
# servo.move_speed(4, 308, 200)
# servo.move_speed(5, 701, 200)


# servo.move_speed(21, 520, 2000)
# servo.move(3, 350)
id = 20
servo.move_speed(id, 100, 1500)
sleep(2)
while True:
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    print("Current Position:\t" + str(servo.get_position(id)) + "\tCurrent Load:\t" + str(servo.ax12.read_load(id)))
    sleep(9)
    servo.move_speed(id, 100, 0)
# print(servo.get_load(21))
