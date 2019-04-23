#!/usr/bin/env python
import rospy
from robot_controller.msg import AudioState
from audio.AudioAnalyzer import AudioAnalyzer as AudioAnalyzer


class AudioPublisher:

    def __init__(self):

        rospy.init_node('audio_publisher', anonymous=True)
        self.audio_publisher = rospy.Publisher('audio', AudioState, queue_size=1)

        self.audioanalyzer = AudioAnalyzer()

        self.audio_rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            self.update_audio()

    def update_audio(self):

        audio_message = AudioState()
        audio_message.header.stamp = rospy.Time.now()

        band_strength = [self.audioanalyzer.low_max, self.audioanalyzer.mid_max, self.audioanalyzer.high_max]

        rospy.logout("low: " + str(band_strength[0]) + " "
                     + "mid: " + str(band_strength[1]) + " "
                     + "high: " + str(band_strength[2]))

        audio_message.band_strength = band_strength
        audio_message.bpm = self.audioanalyzer.bpm

        self.audio_publisher.publish(audio_message)
        self.audio_rate.sleep()


if __name__ == '__main__':
    try:
        audio_pub = AudioPublisher()
    except rospy.ROSInterruptException:
        pass
