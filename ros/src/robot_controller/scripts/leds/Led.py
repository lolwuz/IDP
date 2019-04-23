from neopixel import *
import time


class Led(Adafruit_NeoPixel):

    # Init
    def __init__(self, brightness=255, leds=56, pin=18, frequency=800000, dma=10, invert=False, channel=0):
        Adafruit_NeoPixel.__init__(self, leds, pin, frequency, dma, invert, brightness, channel)

        self.vu = self.VUMeter(self, range(0, 15))
        self.face = self.Face(self, range(15, 44))
        self.legs = self.Legs(self, range(44, 56))

        self.begin()

        self.start_check()

    def start_check(self):
        # Start blink
        for i in range(3):
            for j in range(self.numPixels()):
                self.setPixelColorRGB(j, 0, 255, 0)
            self.show()
            time.sleep(0.15)

            for j in range(self.numPixels()):
                self.setPixelColorRGB(j, 0, 0, 0)
            self.show()
            time.sleep(0.15)

    # Face class
    class Face:
        def __init__(self, strip, leds):
            self.strip = strip
            self.leds = leds

            self.mouth = self.Mouth(self.strip, self.leds[:11])
            self.righteye = self.RightEye(self.strip, self.leds[11:20])
            self.lefteye = self.LeftEye(self.strip, self.leds[20:29])

        def all(self, r, g, b):
            self.execute(r, g, b, range(self.leds))

        def execute(self, r, g, b, included):
            for i in range(self.leds):
                if i in included:
                    self.strip.setPixelColor(i, Color(g, r, b))
            self.strip.show()

        class Mouth:
            def __init__(self, strip, leds):
                self.strip = strip
                self.leds = leds

            def smile(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[2],
                    self.leds[3],
                    self.leds[4]
                ]
                self.execute(r, g, b, included)

            def open(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[2],
                    self.leds[3],
                    self.leds[4],
                    self.leds[8],
                    self.leds[9],
                    self.leds[10]
                ]
                self.execute(r, g, b, included)

            def neutral(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[4],
                    self.leds[5],
                    self.leds[6],
                    self.leds[7]
                ]
                self.execute(r, g, b, included)

            def sad(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[4],
                    self.leds[8],
                    self.leds[9],
                    self.leds[10]
                ]
                self.execute(r, g, b, included)

            def execute(self, r, g, b, included):
                for i in self.leds:
                    if i in included:
                        self.strip.setPixelColor(i, Color(g, r, b))
                    else:
                        self.strip.setPixelColor(i, Color(0, 0, 0))
                self.strip.show()

        class RightEye:
            def __init__(self, strip, leds):
                self.strip = strip
                self.leds = leds

            def open(self, r, g, b):
                included = [
                    self.leds[1],
                    self.leds[3],
                    self.leds[4],
                    self.leds[5],
                    self.leds[7]
                ]
                self.execute(r, g, b, included)

            def square(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[2],
                    self.leds[3],
                    self.leds[5],
                    self.leds[6],
                    self.leds[7],
                    self.leds[8]
                ]
                self.execute(r, g, b, included)

            def bottom_left(self, r, g, b):
                included = [
                    self.leds[3],
                    self.leds[4],
                    self.leds[6],
                    self.leds[7],
                ]
                self.execute(r, g, b, included)

            def bottom_right(self, r, g, b):
                included = [
                    self.leds[4],
                    self.leds[5],
                    self.leds[7],
                    self.leds[8],
                ]
                self.execute(r, g, b, included)

            def top_left(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[3],
                    self.leds[4],
                ]
                self.execute(r, g, b, included)

            def top_right(self, r, g, b):
                included = [
                    self.leds[1],
                    self.leds[2],
                    self.leds[4],
                    self.leds[5],
                ]
                self.execute(r, g, b, included)

            def all(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[2],
                    self.leds[3],
                    self.leds[4],
                    self.leds[5],
                    self.leds[6],
                    self.leds[7],
                    self.leds[8],
                ]
                self.execute(r, g, b, included)

            def sad(self, r, g, b):
                included = [
                    self.leds[1],
                    self.leds[3],
                    self.leds[5]
                ]
                self.execute(r, g, b, included)

            def closed(self, r, g, b):
                included = [
                    self.leds[3],
                    self.leds[4],
                    self.leds[5]
                ]
                self.execute(r, g, b, included)

            def execute(self, r, g, b, included):
                for i in self.leds:
                    if i in included:
                        self.strip.setPixelColor(i, Color(g, r, b))
                    else:
                        self.strip.setPixelColor(i, Color(0, 0, 0))
                self.strip.show()

        class LeftEye:
            def __init__(self, strip, leds):
                self.strip = strip
                self.leds = leds

            def open(self, r, g, b):
                included = [
                    self.leds[1],
                    self.leds[2],
                    self.leds[4],
                    self.leds[5],
                    self.leds[6]
                ]
                self.execute(r, g, b, included)

            def square(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[2],
                    self.leds[3],
                    self.leds[5],
                    self.leds[6],
                    self.leds[7],
                    self.leds[8]
                ]
                self.execute(r, g, b, included)

            def bottom_left(self, r, g, b):
                included = [
                    self.leds[3],
                    self.leds[4],
                    self.leds[6],
                    self.leds[7],
                ]
                self.execute(r, g, b, included)

            def bottom_right(self, r, g, b):
                included = [
                    self.leds[4],
                    self.leds[5],
                    self.leds[7],
                    self.leds[8],
                ]
                self.execute(r, g, b, included)

            def top_left(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[3],
                    self.leds[4],
                ]
                self.execute(r, g, b, included)

            def top_right(self, r, g, b):
                included = [
                    self.leds[1],
                    self.leds[2],
                    self.leds[4],
                    self.leds[5],
                ]
                self.execute(r, g, b, included)

            def all(self, r, g, b):
                included = [
                    self.leds[0],
                    self.leds[1],
                    self.leds[2],
                    self.leds[3],
                    self.leds[4],
                    self.leds[5],
                    self.leds[6],
                    self.leds[7],
                    self.leds[8]
                ]
                self.execute(r, g, b, included)

            def sad(self, r, g, b):
                included = [
                    self.leds[1],
                    self.leds[3],
                    self.leds[5]
                ]
                self.execute(r, g, b, included)

            def closed(self, r, g, b):
                included = [
                    self.leds[3],
                    self.leds[4],
                    self.leds[5]

                ]
                self.execute(r, g, b, included)

            def execute(self, r, g, b, included):
                for i in self.leds:
                    if i in included:
                        self.strip.setPixelColor(i, Color(g, r, b))
                    else:
                        self.strip.setPixelColor(i, Color(0, 0, 0))
                self.strip.show()

    # VU meter class
    class VUMeter:

        def __init__(self, strip, leds):
            self.strip = strip
            self.leds = leds

            self.lowband = self.LowBand(self.strip, self.leds[:5])
            self.midband = self.MidBand(self.strip, list(reversed(self.leds[5:10])))
            self.highband = self.HighBand(self.strip, self.leds[10:15])

        class LowBand:

            def __init__(self, strip, leds):
                self.strip = strip
                self.leds = leds
                self.settings = [0, 0, 0, 0, 0]

            def animate(self, value):
                pass

        class MidBand:

            def __init__(self, strip, leds):
                self.strip = strip
                self.leds = leds
                self.settings = [0, 0, 0, 0, 0]

            def animate(self, value):
                pass

        class HighBand:

            def __init__(self, strip, leds):
                self.strip = strip
                self.leds = leds
                self.settings = [0, 0, 0, 0, 0]

            def animate(self, value):
                pass

        def first_row(self, r, g, b):
            included = [0, 9, 10]
            self.execute(r, g, b, included)

        def second_row(self, r, g, b):
            included = [1, 8, 11]
            self.execute(r, g, b, included)

        def third_row(self, r, g, b):
            included = [2, 7, 12]
            self.execute(r, g, b, included)

        def fourth_row(self, r, g, b):
            included = [3, 6, 13]
            self.execute(r, g, b, included)

        def fifth_row(self, r, g, b):
            included = [4, 5, 14]
            self.execute(r, g, b, included)

        def execute(self, r, g, b, included):
            for i in self.leds:
                if i in included:
                    self.strip.setPixelColor(i, Color(g, r, b))
                else:
                    self.strip.setPixelColor(i, Color(0, 0, 0))
            self.strip.show()

    # Leg class
    class Legs:

        def __init__(self, strip, leds):
            self.strip = strip
            self.leds = leds

        def right_back(self, r, g, b):
            leds = self.leds[0:2]
            self.execute(r, g, b, leds)

        def right_mid(self, r, g, b):
            leds = self.leds[2:4]
            self.execute(r, g, b, leds)

        def right_front(self, r, g, b):
            leds = self.leds[4:6]
            self.execute(r, g, b, leds)

        def left_back(self, r, g, b):
            leds = self.leds[6:8]
            self.execute(r, g, b, leds)

        def left_mid(self, r, g, b):
            leds = self.leds[8:10]
            self.execute(r, g, b, leds)

        def left_front(self, r, g, b):
            leds = self.leds[10:12]
            self.execute(r, g, b, leds)

        def all(self, r, g, b):
            self.execute(r, g, b, self.leds)

        def execute(self, r, g, b, included):
            for i in included:
                self.strip.setPixelColor(i, Color(g, r, b))
            self.strip.show()

    # Disco mode
    def disco(self):
        pass

    # Set all leds to the same constant color
    def all(self, r, g, b):
        all_leds = range(15, 44) + range(44, 56)  # all leds except VU
        for i in range(self.numPixels()):
            if i in all_leds:
                self.setPixelColor(i, Color(g, r, b))
        self.show()

    # Reset all leds (turn all leds off)
    def reset_all(self):
        for i in range(self.numPixels()):
            self.setPixelColor(i, Color(0, 0, 0))
        self.show()
