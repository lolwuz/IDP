#!/usr/bin/env python

import subprocess
import rospy
import time
from joystick.msg import JoystickState
from bluetooth import *
from std_msgs.msg import String
import pickle


class BluetoothNode:

    _MAC = "B8:27:EB:75:44:2C"
    _UUID = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
    _PORT = 3

    def __init__(self):
        # subprocess.call("./btconn.sh")
        rospy.init_node('bluetooth_publisher', anonymous=True)
        service_matches = find_service(uuid=self._UUID, address=None)

        if len(service_matches) == 0:
            print("couldn't find the SampleServer service =(")
            return

        first_match = service_matches[0]
        port = first_match["port"]
        name = first_match["name"]
        host = first_match["host"]

        print("connecting to \"%s\" on %s" % (name, host))

        self.socket = BluetoothSocket(RFCOMM)

        print("Attempting to send message to " + self._MAC + " on port:" + str(self._PORT))

        self.socket.connect((host, port))

        # Start subscriber on connect
        rospy.Subscriber('joystick', JoystickState, self.joystick_callback)

    def joystick_callback(self, data):
        message = pickle.dumps(data)

        if len(data) == 0:
            return
        self.socket.send(message)

if __name__ == '__main__':
    try:
        bluetooth_node = BluetoothNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
