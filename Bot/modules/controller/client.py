from bluetooth import *

def sendMessageTo(targetBluetoothMacAddress):
  targetBluetoothMacAddress="B8:27:EB:11:11:42"
  port = 3
  print("Attempting to send message to " + targetBluetoothMacAddress + " on port:" + str(port))
  sock=BluetoothSocket( RFCOMM )
  sock.connect((targetBluetoothMacAddress, port))
  sock.send("hello!!")
  print(sock.recv(1024))
  sock.close()

sendMessageTo("")