#!/usr/bin/env python
import roslib; roslib.load_manifest('mitsubishi_arm')
import rospy
import serial
import time

#COM.baudrate=115200


if __name__=="__main__":
  rospy.init_node('cyber_glove_teleop')
  init_pos=[0.5,0.5,0.0,0.2,-0.4,1.5]

j = 
  COM =serial.Serial('/dev/ttyUSB0',9600)
  COM.parity = 'E'
  COM.stopbits = 2
  print COM
  buffer = []
  r=rospy.Rate(10) # 10 hz
  COM.write("1\r\n") # tells slave to init
  while 1:
    A = COM.read(1)
    buffer.append(A)
    if A == '\n':
      if "A" in buffer:
        print "GOT an A: ", buffer
        S = str(j[0]) + ',' + str(j[1]) + ',' + str(j[2]) + ',' + str(j[3]) + ',' + str(j[4]) + ',' + str(j[5]) + "\r\n"
        print "SENDING:",S
        COM.write(S)
        break
      print buffer
      buffer = []
    r.sleep()
print "FINISH, i will close the serial port now"
COM.close()
	
