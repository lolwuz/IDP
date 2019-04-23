#run with args -a 1/2/3
#1: search for devices
#2:open port
#3:send message
#is hard coded check code

# Uses Bluez for Linux
#
# sudo apt-get install bluez python-bluez
#
# Taken from: https://people.csail.mit.edu/albert/bluez-intro/x232.html
# Taken from: https://people.csail.mit.edu/albert/bluez-intro/c212.html

from bluetooth import *
import argparse
import datetime
import json
from pprint import pprint
from StringIO import StringIO

server_sock =BluetoothSocket(RFCOMM)

ap = argparse.ArgumentParser()

ap.add_argument("-a", "--argument", required=True, help="mode to run in", default=1)
ap.add_argument("-i", "--ipadress", required=False, help="adress to connect to")
ap.add_argument("-p", "--port", required=False, help="port to connect to")
ap.add_argument("-mac", "--macadress", required=False, help="mac address")
ap.add_argument("-msg", "--message", required=False, help="message to send")

args = vars(ap.parse_args())


def bindBluetoothConnection():
    try:
        server_sock = BluetoothSocket(RFCOMM)

        port = 3
        server_sock.bind(("B8:27:EB:9B:5F:C0", port))
        server_sock.listen(1)
        print("server is listening on port: " + str(port))
    except:
        print("something went wrong")

    return server_sock


def receiveMessages(ssock):
    # server_sock=BluetoothSocket( RFCOMM )

    # port = 3
    # server_sock.bind(("B8:27:EB:9B:5F:C0",port))
    # server_sock.listen(1)
    # print("listening on port " + str(port))
    print("connection is " + str(ssock))
    client_sock, address = ssock.accept()
    print "Accepted connection from " + str(address)

    # while server_sock.accept():
    # data = client_sock.recv(1024)
    # print("received [%s]" % data
    try:
        client_sock.getpeername()

    except:
        print("running bluetooth receive bash script btrecv.sh")
        subprocess.call("./btrecv.sh")

    data = client_sock.recv(1024)
    d = json.loads(data)
    # d = json.loads(io)
    print(d)
    print(d[0]['x'])
    # print(d["glossary"]["title"])
    print (str(datetime.datetime.now().time()) + "received [%s]" % data)
    client_sock.close()
    # server_sock.rebind()
    receiveMessages(ssock)
    #client_sock.close()
    #server_sock.close()

def sendMessageTo(targetBluetoothMacAddress):
  targetBluetoothMacAddress=""
  port = 3
  print("Attempting to send message to " + targetBluetoothMacAddress + " on port:" + str(port))
  sock=BluetoothSocket( RFCOMM )
  sock.connect((targetBluetoothMacAddress, port))
  sock.send("topic")
  print(sock.recv(1024))
  sock.close()


def lookUpNearbyBluetoothDevices():
    print("looking for nearby bluetooth devices..")
    nearby_devices = discover_devices()
    for bdaddr in nearby_devices:
        print str(lookup_name(bdaddr)) + " [" + str(bdaddr) + "]"


print("running with " + str(args["argument"]))
if (str(args["argument"]) == "1"):
    lookUpNearbyBluetoothDevices()
elif (args["argument"] == "2"):
    receiveMessages()
elif (args["argument"] == "3"):
    sendMessageTo('a')

