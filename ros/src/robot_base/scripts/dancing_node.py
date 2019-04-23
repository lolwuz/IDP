#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from dance.motor_dict import motor_dict
from dance.move_dict import move_dict
from dance.led_dict import led_dict
from base.base_node import BaseNode


class DancingNode(BaseNode):
    def __init__(self):
        """ Node for controlling the dance """
        super(DancingNode, self).__init__("dancing", False, 24)

        self.dict_list = [led_dict, motor_dict, move_dict]

        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self.is_running:
                self.update()

        rospy.spin()

    def update(self):
        """ Update loop """
        super(DancingNode, self).update()

        current_time = rospy.get_time()
        time_elapsed = current_time - self.start_time

        self.check_move(time_elapsed)
        self.check_led(time_elapsed)
        self.check_motor(time_elapsed)

    def check_move(self, time_elapsed):
        for name in move_dict:
            move = move_dict[name]

            if move["time"] <= time_elapsed:
                if name not in self.move_array:
                    self.set_goal_position(move["id_array"], move["po_array"], move["sp_array"])
                    self.move_array.append(name)

    def check_led(self, time_elapsed):
        for name in led_dict:
            led = led_dict[name]

            if led["time"] <= time_elapsed:
                if name not in self.led_array:
                    self.set_led_function(led["rgb"], led["section"], led["part"], led["function"])
                    self.led_array.append(name)

    def check_motor(self, time_elapsed):
        for name in motor_dict:
            motor = motor_dict[name]

            if motor["time"] <= time_elapsed:
                if name not in self.motor_array:
                    rospy.logout(motor)
                    self.change_motor_state(motor["motors"], motor["direction"])

                    if motor["pwm"] != 0:
                        self.change_pwm_state(motor["pwm"])

                    self.motor_array.append(name)





if __name__ == '__main__':
    try:
        dancing_node = DancingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
