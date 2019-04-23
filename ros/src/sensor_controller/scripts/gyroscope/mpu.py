import smbus
import math


class Mpu:

    _ADDRESS = 0x68  # This is the address value read via the i2cdetect command

    _POWER_MANAGEMENT_1 = 0x6b
    _POWER_MANAGEMENT_2 = 0x6c

    def __init__(self):

        self.acceleration_x = 0.0
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0
        self.acceleration_x_scaled = 0.0
        self.acceleration_y_scaled = 0.0
        self.acceleration_z_scaled = 0.0

        self.gyroscope_x = 0.0
        self.gyroscope_y = 0.0
        self.gyroscope_z = 0.0

        self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards
        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(self._ADDRESS, self._POWER_MANAGEMENT_1, 0)  # power management 3.3v

    def update(self):
        self.acceleration_x = self.read_word_2c(0x3B)
        self.acceleration_y = self.read_word_2c(0x3D)
        self.acceleration_z = self.read_word_2c(0x3F)

        self.acceleration_x_scaled = self.acceleration_x / 16384.0
        self.acceleration_y_scaled = self.acceleration_y / 16384.0
        self.acceleration_z_scaled = self.acceleration_z / 16384.0

        self.gyroscope_x = self.read_word_2c(0x43)
        self.gyroscope_y = self.read_word_2c(0x45)
        self.gyroscope_z = self.read_word_2c(0x47)

    def read_byte(self, adr):
        return self.bus.read_byte_data(self._ADDRESS, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self._ADDRESS, adr)
        low = self.bus.read_byte_data(self._ADDRESS, adr + 1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def get_rotation_y(self):
        radians = math.atan2(self.acceleration_x_scaled,
                             self.dist(self.acceleration_y_scaled, self.acceleration_z_scaled))
        return -math.degrees(radians)

    def get_rotation_x(self):
        radians = math.atan2(self.acceleration_y_scaled,
                             self.dist(self.acceleration_x_scaled, self.acceleration_z_scaled))
        return math.degrees(radians)

    @staticmethod
    def dist(a, b):
        return math.sqrt((a * a) + (b * b))