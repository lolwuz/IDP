import rospy
from base.base_node import BaseNode
from robot_controller.msg import AudioState


class LineNode(BaseNode):

    def __init__(self):
        """
        Node for controlling the line-dance
        """
        super(LineNode, self).__init__("line", False, 24)

        rospy.Subscriber('audio', AudioState, self.audio_callback)
        # rospy.Subscriber('sound', SoundState, self.sound_callback)  # Joystick subscriber
        # rospy.Subscriber('line', LineState, self.line_callback)  # Joystick subscriber

        while not rospy.is_shutdown() and self.is_running:
            self.update()

    def update(self):
        """ Update loop """
        super(LineNode, self).update()

    def audio_callback(self):
        pass

    def line_callback(self):
        pass







