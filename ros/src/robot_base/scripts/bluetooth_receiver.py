#!/usr/bin/env python
import subprocess
from bluetooth import *
from joystick.msg import JoystickState
import rospy
import pickle


class BluetoothClient:

    _MAC = "B8:27:EB:75:44:2C"
    _PORT = 3
    _UUID = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

    def __init__(self):
        """ Receives serialized bluetooth messages and publishes them """
        # subprocess.call("btconn.sh", shell=True)

        self.blue_publisher = rospy.Publisher('joystick', JoystickState, queue_size=1)
        rospy.init_node('bluetooth_receiver', anonymous=True)

        self.sock = BluetoothSocket(RFCOMM)
        self.sock.bind((self._MAC, self._PORT))
        self.sock.listen(1)

        self.port = self.sock.getsockname()[1]

        advertise_service(self.sock, "SampleServer", service_id=self._UUID, service_classes=[self._UUID, SERIAL_PORT_CLASS], profiles=[SERIAL_PORT_PROFILE])

        print("Waiting for connection on RFCOMM channel %d" % self.port)

        conn, addr = self.sock.accept()
        rospy.logout("Socket accept")

        try:
            while True:
                data = self.sock.recv(1024)
                if len(data) == 0:
                    break
                print("received [%s]" % data)
                self.publish_message(data)
        except IOError:
            pass

    def publish_message(self, data):
        """
        Publish message to the correct topic
        :param data: ros/message
        :return:
        """
        deserialized = pickle.loads(data)
        self.blue_publisher.publish(deserialized)


if __name__ == '__main__':
    try:
        bluetooth_client = BluetoothClient()
    except rospy.ROSInterruptException:
        pass
