import sys
from lib.servo import Servo
<<<<<<< HEAD
import time
import Tkinter


class Test:
    def __init__(self):
        self.servo = Servo()
        self.id1 = 20
        self.id2 = 21
        self.standard_position = 700
        self.print_load()
        # self.clamp_outwards()
        # self.clamp_up()
        # self.clamp_inwards()
        # self.clamp_down()

    def clamp_down(self):
        self.servo.ax12.move_speed(self.id1, self.standard_position, 1800)
        time.sleep(3)
        while True:
            print("Current load servo 20:" + " " + str(self.servo.ax12.read_load(self.id1)))
            print("-----------------------------")
            time.sleep(1.5)
            if self.servo.ax12.read_load(self.id1) > 10 and self.servo.ax12.read_load(self.id1) < 1570:
                self.servo.move_speed(self.id1, 100, 0)
                print(str(self.servo.ax12.read_load(self.id2)))
                print("Clamped!")
                # self.clamp_up()
                break

    def clamp_inwards(self):
        self.servo.ax12.move_speed(self.id2, self.standard_position, 800)
        time.sleep(3)
        while True:
            print("Current load servo 21:" + " " + str(self.servo.ax12.read_load(self.id2)))
            print("-----------------------------")
            time.sleep(1.5)
            if self.servo.ax12.read_load(self.id2) > 1000 and self.servo.ax12.read_load(self.id2) < 2000:
                self.servo.move_speed(self.id2, 100, 0)
                print(str(self.servo.ax12.read_load(self.id2)))
                print("Clamped!")
                break

    def clamp_up(self):
        self.servo.ax12.move_speed(self.id1, self.standard_position, 800)
        time.sleep(3)
        while True:
            print("Current load servo 20:" + " " + str(self.servo.ax12.read_load(self.id1)))
            print("-----------------------------")
            time.sleep(0.3)
            if self.servo.ax12.read_load(self.id1) > 1800 and self.servo.ax12.read_load(self.id1) < 2000:
                self.servo.move_speed(self.id1, self.standard_position, 0)
                print("Completed")
                break

    def clamp_outwards(self):
        self.servo.ax12.move_speed(self.id2, self.standard_position, 1800)
        time.sleep(3)
        while True:
            print("Current load servo 21:" + " " + str(self.servo.ax12.read_load(self.id2)))
            print("-----------------------------")
            time.sleep(10)
            self.servo.move_speed(self.id2, 100, 0)
            break
            # if self.servo.ax12.read_load(self.id2) > 500:
            #     self.servo.move_speed(self.id2, 100, 0)
            #     print("Clamped!")
            #     break

    def print_load(self):
        self.servo.move_speed(self.id2, 1023, 1000)
        while True:
            x = 0
            count = 0
            for i in range(1, 100):
                load = self.servo.ax12.read_load(self.id2)
                # position = self.servo.ax12.read_position(self.id2)
                if 0 < load < 1023:
                    x += load
                    count += 1
                    print("i\t" + str(i) + "\tload\t" + str(load))
                    if load < 40:
                        try:
                            self.servo.move_speed(self.id2, 1023, 0)
                            print("Stopped servo")
                        except:
                            continue

                        break
                time.sleep(0.75)
            print("Load" + str(x / count))


if __name__ == '__main__':
    test = Test()
# =======
from lib.servo_dict import servo_dict

servo = Servo()

id_array = []
position_array = []
speed_array = []

hoekverdraaing = 27.8

speed = 600

speed_A = 321.1978896
speed_B = 835.114513


def test_split():
    for name in servo_dict:
        servo_obj = servo_dict[name]

        servo_id = servo_obj["id"]
        servo_min = servo.dxl_angle_to_degrees(servo_obj["min"])
        servo_max = servo.dxl_angle_to_degrees(servo_obj["max"])

        # print(name)

        factor = get_factor(name)

        servo_angle = servo_min + hoekverdraaing * factor
        dxl_angle = servo.degrees_to_dxl_angle(servo_angle)

        id_array.append(servo_id)
        position_array.append(int(dxl_angle))
        speed_array.append(50)

    set_servo_state(id_array, position_array, speed_array)


def get_factor(name):
    factor = 0
    if name.endswith("a"):
        if name.split("_")[2].split("a")[0] == 'l':
            factor = 1
            print("Joint: " + name + " I am L and factor = 1")
        elif name.split("_")[2].split("a")[0] == 'r':
            factor = -1
            print("Joint: " + name + " I am R and factor = -1")
    if name.endswith("b"):
        if name.split("_")[2].split("b")[0] == 'l':
            factor = 1
            print("Joint: " + name + " I am L and factor = 1")
        elif name.split("_")[2].split("b")[0] == 'r':
            factor = -1
            print("Joint: " + name + " I am R and factor = +1")
    if name.endswith("c"):
        if name.split("_")[2].split("c")[0] == 'l':
            factor = -1
            print("Joint: " + name + " I am L and factor = -1")
        elif name.split("_")[2].split("c")[0] == 'r':
            factor = 1
            print("Joint: " + name + " I am R and factor = 1")

    return factor


def set_servo_state(id_array, position_array, speed):
    """
    Checks if supplied positions are within range to prevent ax12 servo overload
    :param id_array:
    :param position_array:
    :param speed:
    :return:
    """
    for name in servo_dict:
        servo_obj = servo_dict[name]

        for i in range(len(id_array)):
            if servo_obj["id"] == id_array[i]:
                servo_max = servo_obj["max"]
                servo_min = servo_obj["min"]

                if servo_min < servo_max:
                    range_servo = range(servo_min, servo_max + 1)
                else:
                    range_servo = range(servo_max, servo_min + 1)

                if position_array[i] not in range_servo:
                    print(servo_obj["id"], position_array[i])
                    return

    servo.sync_write_move_speed(id_array, position_array, speed)

test_split()
# >>>>>>> ROS
