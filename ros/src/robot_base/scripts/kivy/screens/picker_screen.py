from kivy.uix.screenmanager import Screen


class PickerScreen(Screen):
    def __init__(self, **kw):
        super(PickerScreen, self).__init__(**kw)
        # self.led_publisher = rospy.Publisher('set_goal_position', Led, queue_size=1)
