#!/usr/bin/env python
from pynamixel.protocol1_0 import IO
from pynamixel.ports import *
import time
import rospy
from pynamixel.msg import Actuation

#buffer = []

def actuate(data):
    global dxl_io
    ids = map(int,data.ids)
    angles = data.angles
    speeds = map(int,data.speeds)
    #rospy.loginfo(data)
    angles = list(angles)
    #ids.extend([19])
    #angles.extend([0.0])
    if rospy.get_param('/writer/debug') == False:
        #print("writing")
        if len(ids) == len(angles) and len(ids) == len(speeds):
            #dxl_io.set_moving_speed(dict(zip(ids,speeds)))
            print(dict(zip(ids,angles)))
            dxl_io.write(dict(zip(ids,angles)))
            #time.sleep(0.01)
    else:
        angles = [round(i,2) for i in angles]    
        print(dict(zip(ids,angles)))
    

def start():
    rospy.init_node('writer', anonymous=False)
    ids = range(1,19)
    angles = [0 for id in ids]
    dxl_io.set_torque_status(ids,1)
    raw_input("Init ?")
    dxl_io.write(dict(zip(ids,angles)))
    raw_input("Start ?")
    rospy.Subscriber('actuation', Actuation, actuate)
    rospy.spin()

if __name__ == '__main__': 
    port = list_port()[0]
    print(port)
    dxl_io = IO(baudrate=1000000, port=port)
    start()
