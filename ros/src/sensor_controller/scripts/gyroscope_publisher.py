#!/usr/bin/env python
import rospy
from sensor_controller.msg import GyroState
from gyroscope.mpu import Mpu


class GyroScopePublisher:

    def __init__(self):
        self.gyroscope = Mpu()
        self.gyroscope_publisher = rospy.Publisher('gyroscope_robot', GyroState, queue_size=1)
        rospy.init_node('gyroscope_publisher', anonymous=True)
        self.gyro_rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # Update the state of the mpu
            self.gyroscope.update()
            self.update_pwm()

    def update_pwm(self):
        msg = GyroState()
        msg.header.stamp = rospy.Time.now()
        msg.acceleration = [
            self.gyroscope.acceleration_x,
            self.gyroscope.acceleration_y,
            self.gyroscope.acceleration_z
        ]

        msg.rotation = [
            self.gyroscope.get_rotation_x(),
            self.gyroscope.get_rotation_y(),
            0
        ]

        rospy.loginfo(msg)
        self.gyroscope_publisher.publish(msg)
        self.gyro_rate.sleep()


if __name__ == '__main__':
    try:
        giro_pub = GyroScopePublisher()
    except rospy.ROSInterruptException:
        pass
