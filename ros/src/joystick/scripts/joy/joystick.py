import smbus
import math
import RPi.GPIO as gpio
import time

# 13, 19, 26
class Joy():

    def __init__(self, name):
        self.name = name
        self.position = {'x': 0, 'y': 0}
        self.press_button = False

    def set_joy(self, raw_x, raw_y):
        self.position['x'] = self.row_to12bits(raw_x)
        self.position['y'] = self.row_to12bits(raw_y)

    def set_button(self, button_state):
        self.press_button = button_state

    def row_to12bits(self, data):
        # Convert the data to 12-bits
        raw_adc = (data[0] & 0x0F) * 256 + data[1]
        if raw_adc > 2047:
            raw_adc -= 4095
        return raw_adc


class Joystick:

    def __init__(self, i2c_address, button_pin_1, button_pin_2):
        # Define 2 joy controls
        self.joys = [Joy('right_joy'), Joy('left_joy')]

        # Set press button input on the GPIO
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)

        gpio.setup(button_pin_1, gpio.IN)
        gpio.setup(button_pin_2, gpio.IN)

        gpio.setup(13, gpio.OUT)
        gpio.setup(19, gpio.OUT)
        gpio.setup(26, gpio.OUT)

        gpio.output(13, gpio.HIGH)
        gpio.output(19, gpio.HIGH)
        gpio.output(26, gpio.HIGH)

        self.i2c_address = i2c_address

        self.button_pin_1 = button_pin_1
        self.button_pin_2 = button_pin_2

        self.bus = smbus.SMBus(1)

    def update(self):
        # Update joy states
        self.bus.write_byte(self.i2c_address, 0x10)
        time.sleep(0.01)
        raw_data_1_y = self.bus.read_i2c_block_data(self.i2c_address, 0x00, 2)
        self.bus.write_byte(self.i2c_address, 0x30)
        time.sleep(0.01)
        raw_data_1_x = self.bus.read_i2c_block_data(self.i2c_address, 0x00, 2)
        self.bus.write_byte(self.i2c_address, 0x50)
        time.sleep(0.01)
        raw_data_2_y = self.bus.read_i2c_block_data(self.i2c_address, 0x00, 2)
        self.bus.write_byte(self.i2c_address, 0x70)
        time.sleep(0.01)
        raw_data_2_x = self.bus.read_i2c_block_data(self.i2c_address, 0x00, 2)
        time.sleep(0.01)

        self.joys[0].set_joy(raw_data_1_x, raw_data_1_y)
        self.joys[1].set_joy(raw_data_2_x, raw_data_2_y)

        # Update buttonjoy state
        input_value_1 = gpio.input(self.button_pin_1)
        input_value_2 = gpio.input(self.button_pin_2)

        if not input_value_1:
            self.joys[0].set_button(True)
        else:
            self.joys[0].set_button(False)

        if not input_value_2:
            self.joys[1].set_button(True)
        else:
            self.joys[1].set_button(False)

    def get_joys(self):
        return self.joys
