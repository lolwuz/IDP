#requires pybluez client pi also needs it
#
#sudo apt-get install bluez python-bluez
#
# run bluetoothctl for mac addresses

from bluetooth import * #requires pybluez


def receiveMessages():
    server_sock = BluetoothSocket(RFCOMM)

    port = 3
    server_sock.bind(("B8:27:EB:11:11:42", port))
    server_sock.listen(1)
    print("listening on port " + str(port))

    client_sock, address = server_sock.accept()
    print "Accepted connection from " + str(address)

    data = client_sock.recv(1024)
    print "received [%s]" % data

    receiveMessages()
    #client_sock.close()
    #server_sock.close()

receiveMessages()