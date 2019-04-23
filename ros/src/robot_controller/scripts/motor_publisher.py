#!/usr/bin/env python
import rospy
from motor_controller.msg import MotorState, PwmState, GiroState
import std_msgs.msg


class MotorPublisher:

    def __init__(self):

        self.pwm = 0
        self.motor_pwm_publisher = rospy.Publisher('motor_pwm', PwmState, queue_size=1)
        self.motor_state_publisher = rospy.Publisher('motor_state', MotorState, queue_size=1)
        rospy.init_node('motor_publisher', anonymous=True)

        rospy.Subscriber('gyroscope_robot', GiroState, self.gyro_callback)

        self.motor_pwm_rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            self.update_pwm()

    def update_pwm(self):

        pwm_message = PwmState()
        pwm_message.header.stamp = rospy.Time.now()
        pwm_message.pwm = self.pwm

        self.motor_pwm_publisher.publish(pwm_message)
        self.motor_pwm_rate.sleep()

    def gyro_callback(self, data):

        percentage = abs(data.rotation[0]) / 100

        rospy.loginfo(percentage)
        self.pwm = percentage * 4095


if __name__ == '__main__':
    try:
        motor_pub = MotorPublisher()
    except rospy.ROSInterruptException:
        pass
