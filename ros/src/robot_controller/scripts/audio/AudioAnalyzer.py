import numpy as np
import pyaudio
import threading


class AudioAnalyzer:

    # Audio Configuration
    FORMAT = pyaudio.paFloat32
    CHANNELS = 1
    RATE = 44100
    CHUNK = 1024
    START = 0
    N = 512

    wave_x = 0
    wave_y = 0
    spec_x = 0
    spec_y = 0
    data = []

    low_max = 0
    mid_max = 0
    high_max = 0
    bpm = 0

    def __init__(self):

        self.pa = pyaudio.PyAudio()

        self.stream = self.pa.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK)

        # Main loop
        self.t1 = threading.Thread(target=self.loop)
        self.t1.start()

        # self.loop()

    def loop(self):
        try:
            while True:
                self.data = self.audioinput()
                self.fft()

        except KeyboardInterrupt:
            self.stream.stop_stream()
            self.stream.close()
            self.pa.terminate()

        print("End...")

    def audioinput(self):
        ret = self.stream.read(self.CHUNK, False)
        ret = np.fromstring(ret, np.float32)
        return ret

    def fft(self):
        # self.wave_x = range(self.START, self.START + self.N)
        # self.wave_y = self.data[self.START:self.START + self.N]
        self.spec_x = np.fft.fftfreq(self.N, d=1.0 / self.RATE)
        y = np.fft.fft(self.data[self.START:self.START + self.N])
        self.spec_y = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in y]

        # print(self.spec_y)
        self.low_max = int(round(max(self.spec_y[257:])) * 25)
        self.mid_max = int(round(max(self.spec_y[26:256])) * 50)
        self.high_max = int(round(max(self.spec_y[:25])) * 50)


if __name__ == "__main__":
    aud = AudioAnalyzer()
