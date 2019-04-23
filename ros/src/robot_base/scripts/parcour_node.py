#!/usr/bin/env python
import rospy
from lib.servo_dict import servo_dict
from base.base_node import BaseNode



class ParcourNode(BaseNode):

    def __init__(self):
        super(ParcourNode, self).__init__("parcour", True, 24)
        self.start_time = rospy.get_time()

        while not rospy.is_shutdown():
            self.update()

        rospy.spin()

    def set_servo(self):
        id_array = []
        position_array = []
        speed_array = []

        hoekverdraaing = 85

        speed_up = 139
        speed_down = 35

        for name in servo_dict:
            servo_obj = servo_dict[name]
            servo_id = servo_obj["id"]

            if servo_obj["def"] == "suspension":
                if servo_id == 4 or servo_id == 5:
                    id_array.append(servo_id)
                    position_array.append(int(hoekverdraaing))
                    speed_array.append(speed_down)
                if servo_id == 6 or servo_id == 7:
                    id_array.append(servo_id)
                    position_array.append(int(servo_obj["min"]))
                    speed_array.append(speed_down)
                if servo_id == 8 or servo_id == 9:
                    id_array.append(servo_id)
                    position_array.append(int(hoekverdraaing))
                    speed_array.append(speed_down)

            else:
                id_array.append(servo_id)
                position_array.append(int((servo_obj["min"] + servo_obj["max"]) / 2))
                speed_array.append(speed_down)

        self.set_goal_position(id_array, position_array, speed_array)

    def update(self):
        super(ParcourNode, self).update()

        self.change_motor_state("all", "down")
        current_time = rospy.get_time()
        elapsed_time = current_time - self.start_time






        for name in servo_dict:

            if servo_obj["def"] == "suspension":
                if servo_id == 4 or servo_id == 5:
                    id_array.append(servo_id)
                    position_array.append(int(servo_obj["min"]))
                    speed_array.append(speed_down)
                if servo_id == 6 or servo_id == 7:
                    id_array.append(servo_id)
                    position_array.append(int(servo_obj["min"]))
                    speed_array.append(speed_down)
                if servo_id == 8 or servo_id == 9:
                    id_array.append(servo_id)
                    position_array.append(int(servo_obj["min"]))
                    speed_array.append(speed_down)

            else:
                id_array.append(servo_id)
                position_array.append(int((servo_obj["min"] + servo_obj["max"]) / 2))
                speed_array.append(speed_down)

        self.set_goal_position(id_array, position_array, speed_array)

        self.change_pwm_state(1000)

    def get_factor(self, name):
        factor = 0
        if name.endswith("a"):
            if name.split("_")[2].split("a")[0] == 'l':
                factor = 1
            elif name.split("_")[2].split("a")[0] == 'r':
                factor = -1
        if name.endswith("b"):
            if name.split("_")[2].split("b")[0] == 'l':
                factor = 1
            elif name.split("_")[2].split("b")[0] == 'r':
                factor = -1
        if name.endswith("c"):
            if name.split("_")[2].split("c")[0] == 'l':
                factor = -1
            elif name.split("_")[2].split("c")[0] == 'r':
                factor = 1

        return factor


if __name__ == '__main__':
    try:
        parcour_node = ParcourNode()
    except rospy.ROSInterruptException:
        pass
