#!/usr/bin/env python
#****************************************************************************
# Copyright (C) 2013 - 2014 by                                              *
# Tarek Taha and Rui Figueiredo, Khalifa University Robotics Institute KURI *
# <tarek.taha@kustar.ac.ae> <rui.defigueiredo@kustar.ac.ae>                 *
#                                                                           *
# 									    *
# This program is free software; you can redistribute it and/or modify      *
# it under the terms of the GNU General Public License as published by      *
# the Free Software Foundation; either version 2 of the License, or         *
# (at your option) any later version. 					    *
# 									    *
# This program is distributed in the hope that it will be useful, 	    *
# but WITHOUT ANY WARRANTY; without even the implied warranty of 	    *
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 		    *
# GNU General Public License for more details. 				    *
# 									    *
# You should have received a copy of the GNU General Public License 	    *
# along with this program; if not, write to the 			    *
# Free Software Foundation, Inc., 					    *
# 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA. 		    *
#****************************************************************************/

import roslib
import rospy
from geometry_msgs.msg import Twist
from evdev import uinput, ecodes as e

import serial
import time
def ask():
  return raw_input('\nInsert a valid command: ' )

def read(old):
  r=c.read(1) #read G
  new=[]
  while 1:
    r=c.read(1)
    if ord(r) == 0:
	if len(new) == 18:	  
	   return new
	else:
	   return old 
    else:
	#print ord(r)
	new.append(ord(r))


# TODO: CALIBRATE THE OFFSETS


if __name__=="__main__":
    
    rospy.init_node('cyber_glove_teleop')
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist)

    c=serial.Serial('/dev/ttyUSB0')
    c.baudrate=115200
    data=[]
    circular=[]
    f = open('data.txt', 'w')
    r=rospy.Rate(10) # 10 hz
    while 1:

 	c.write("G")
 	data=read(data)

 	print data
 	t = time.time()
 	#f.write(str(t))
 	#f.write(' ')
 	count = 0
        print data
 	for i in data:
		count = count + i
		#f.write(str(i))
		#f.write(' ')
	print data[5]
	#f.write("\n")
	print count

    	control_linear_speed=0.0
	control_angular_speed=0.0
	if count > 2000:
	  #with uinput.UInput() as ui:
	    #ui.write(e.EV_KEY, e.KEY_LEFTSHIFT, 1)
	    #ui.write(e.EV_KEY, e.KEY_UP, 1)
	    #ui.syn()
	    control_linear_speed=1.0
	    print 'Forward\n'
	elif data[0] > 120:
	  #with uinput.UInput() as ui:
	    #ui.write(e.EV_KEY, e.KEY_DOWN, 1)
	    #ui.syn()
	    control_linear_speed=-1.0
	    print 'Backward\n'
	elif data[5] > 110:
	  #with uinput.UInput() as ui:
	    #ui.write(e.EV_KEY, e.KEY_RIGHT, 1)
	    #ui.syn()
	    control_angular_speed=1.0
	    print 'Right\n'
	elif data[10] > 110:
	  #with uinput.UInput() as ui:
	    #ui.write(e.EV_KEY, e.KEY_LEFT, 1)
	    #ui.syn()
	    control_angular_speed=-1.0
	    print 'Left\n'
	else:
	  #with uinput.UInput() as ui:
	    #ui.write(e.EV_KEY, e.KEY_SPACE, 1)
	    #ui.syn()
	    print 'nothing'
	#f.close()
        twist = Twist()
        twist.linear.x = control_linear_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_speed
        pub.publish(twist)
        r.sleep()
    c.close()
