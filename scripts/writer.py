#!/usr/bin/env python
from pynamixel.dynamixel1_0 import Dynamixel
from pynamixel.ports import *
import time
import rospy
from pynamixel.msg import Actuation

def actuate(data):
    global dxl_io
    ids = map(int,data.ids)
    angles = data.angles
    speeds = map(int,data.speeds)
    rospy.loginfo(dict(zip(ids,angles)))
    if len(ids) == len(angles) and len(ids) == len(speeds):
        dxl_io.set_moving_speed(dict(zip(ids,speeds)))
        dxl_io.write(dict(zip(ids,angles)))
        #time.sleep(0.01)

def start():
    rospy.init_node('writer', anonymous=True)
    rospy.Subscriber('actuation', Actuation, actuate)
    rospy.spin()

if __name__ == '__main__': 
    port = list_port()[0]
    dxl_io = Dynamixel(baudrate=1000000, port=port)
    start()
