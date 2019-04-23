import smbus
import time

from lib.servo import Servo


class TowerGripper:
    def __init__(self):
        self.servo = Servo()
        self.id1 = 20
        self.id2 = 21
        self.standard_position = 700
        self.bus = smbus.SMBus(1)
        self.bus.write_byte(0x68, 0x10)
        try:
            # self.clamp_outwards()
            # self.clamp_up()
            self.clamp_inwards()
            self.clamp_down()
        except:
            self.servo.ax12.move_speed(self.id1, self.standard_position, 0)
            self.servo.ax12.move_speed(self.id2, self.standard_position, 0)

    def clamp_inwards(self):
        self.servo.ax12.move_speed(self.id2, self.standard_position, 800)

        while True:
            data = self.bus.read_i2c_block_data(0x68, 0x00, 2)
            # Convert the data to 12-bits
            time.sleep(0.5)
            raw_adc = (data[0] & 0x0F) * 256 + data[1]
            print(data)
            if 75 < raw_adc < 79:
                self.servo.move_speed(self.id2, 100, 0)
                print("Clamped!")
                break
            print "Digital value of Analog Input : %f" % raw_adc

    def clamp_down(self):
        self.servo.ax12.move_speed(self.id1, self.standard_position, 1800)

        while True:
            data = self.bus.read_i2c_block_data(0x68, 0x00, 2)
            # Convert the data to 12-bits
            time.sleep(0.5)
            raw_adc = (data[0] & 0x0F) * 256 + data[1]
            print(data)
            if raw_adc > 80:
                self.servo.move_speed(self.id1, 100, 0)
                print("Clamped!")
                break
            print "Digital value of Analog Input : %f" % raw_adc

    def clamp_up(self):
        self.servo.ax12.move_speed(self.id1, self.standard_position, 800)
        time.sleep(6)
        self.servo.ax12.move_speed(self.id1, self.standard_position, 0)

    def clamp_outwards(self):
        self.servo.ax12.move_speed(self.id2, self.standard_position, 1800)
        time.sleep(6)
        self.servo.ax12.move_speed(self.id2, self.standard_position, 0)


if __name__ == '__main__':
    towerGripper = TowerGripper()
