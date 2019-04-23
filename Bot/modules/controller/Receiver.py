import subprocess
from bluetooth import *
import argparse
import datetime
import json
from pprint import pprint
from StringIO import StringIO

server_sock =BluetoothSocket(RFCOMM)

def __init__(self):
    subprocess.call("./btrecv.sh")



def receiveMessages(ssock):
    # server_sock=BluetoothSocket( RFCOMM )

    # port = 3
    # server_sock.bind(("B8:27:EB:9B:5F:C0",port))
    # server_sock.listen(1)
    # print("listening on port " + str(port))
    print("connection is " + str(ssock))
    client_sock, address = ssock.accept()
    print "Accepted connection from " + str(address)

    try:
        client_sock.getpeername()

    except:
        print("running bluetooth receive bash script btrecv.sh")
        subprocess.call("./btrecv.sh")

    # while server_sock.accept():
    # data = client_sock.recv(1024)
    # print("received [%s]" % data

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