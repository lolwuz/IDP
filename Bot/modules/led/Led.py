import time
from neopixel import *
from random import randint


class Led(Adafruit_NeoPixel):
    def __init__(self, leds, pin, brightness):
        Adafruit_NeoPixel.__init__(self, leds, pin, 800000, 5, False, brightness)

        self.leds = leds
        self.pin = pin
        self.brightness = brightness

    def wheel(self, pos):
        if pos < 85:
            return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
            pos -= 85
            return Color(255 - pos * 3, 0, pos * 3)
        else:
            pos -= 170
            return Color(0, pos * 3, 255 - pos * 3)

    def set_constant_color(self, r, g, b):  # loopled
        print ("You choose frequency 1")
        for i in range(self.numPixels()):
            self.setPixelColor(i, Color(r, g, b))
            self.show()


    def two(self, r, g, b, wait_ms=50, iterations=10):  # theaterChase
        print ("You choose frequency 2")
        for j in range(iterations):
            for q in range(3):
                for i in range(0, self.numPixels(), 3):
                    self.setPixelColor(i + q, color)
                self.show()
                time.sleep(wait_ms / 500.0)
                for i in range(0, self.numPixels(), 3):
                    self.setPixelColor(i + q, 0)

    def three(self, r, g, b, wait_ms=20, iterations=1):
        print ("You choose frequency 3")
        for j in range(256 * iterations):
            for i in range(self.numPixels()):
                self.setPixelColor(i, self.wheel((i + j) & 255))
            self.show()
            time.sleep(wait_ms / 500.0)

    def resetLeds(self, wait_ms=10):

        for i in range(self.numPixels()):
            self.setPixelColor(i, color)
            self.show()


