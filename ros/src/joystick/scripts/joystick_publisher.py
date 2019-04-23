#!/usr/bin/env python
import rospy
from joystick.msg import JoyState, JoystickState
from joy.joystick import Joystick


class JoystickPublisher:

    def __init__(self):
        """
        JoystickPublisher class pulls the joystick values and publishes them on the joystick_state topic
        """
        self.joystick = Joystick(0x6b, 6, 5) # i2c address 0x6b, buttons on 5, 6 for joy1, joy2
        self.joystick_publisher = rospy.Publisher('joystick', JoystickState, queue_size=1)
        rospy.init_node('joystick_state', anonymous=True)
        self.joystick_rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # Update the state of the joystick
            self.joystick.update()
            # Publish joystick state
            self.update_joystick()

    def update_joystick(self):
        """
        Publishes a joystick state over the joystick_publisher
        """
        joys = self.joystick.get_joys()  # Get Joy positions and buttons

        joystick_msg = JoystickState()  # Array of Joys
        joystick_msg.header.stamp = rospy.Time.now()  # Add a time stamp

        for joy in joys:
            msg_joy = JoyState()
            msg_joy.x = joy.position['x']
            msg_joy.y = joy.position['y']
            msg_joy.button = joy.press_button
            joystick_msg.joys.append(msg_joy)

        self.joystick_publisher.publish(joystick_msg)  # Publish the current joy_state
        self.joystick_rate.sleep()  # Sleep


if __name__ == '__main__':
    try:
        joy_pub = JoystickPublisher()
    except rospy.ROSInterruptException:
        pass
