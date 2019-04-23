#!/usr/bin/expect

spawn "bluetoothctl"
expect "#"
send "power on\r"
expect "Changing power on succeeded"
send "pairable on\r"
expect "Changing pairable on succeeded"
send "agent on\r"
expect "Agent registered"
send "default-agent\r"
expect "Default agent request successful"
send "connect B8:27:EB:9B:5F:C0\r"
expect "d"