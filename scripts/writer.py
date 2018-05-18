#!/usr/bin/env python
from pynamixel.protocol1_0 import Dxl_IO
import pynamixel.ports as ports
import time
import rospy
from pynamixel.msg import Actuation


def actuate(data):
    global dxl_io
    ids = map(int,data.ids)
    angles = data.angles
    speeds = map(int,data.speeds)
    angles = list(angles)
    if rospy.get_param('/writer/debug') == False:
        if len(ids) == len(angles) and len(ids) == len(speeds):
            #dxl_io.set_moving_speed(dict(zip(ids,speeds)))
            dxl_io.set_goal_position(dict(zip(ids,angles)))
            #time.sleep(0.01)
    
    angles = [round(i,2) for i in angles]    
    rospy.loginfo(dict(zip(ids,angles)))
    

def start():
    rospy.init_node('writer', anonymous=False)
    ids = range(1,19)
    angles = [0 for id in ids]
    dxl_io.set_torque_status(ids,1)
    raw_input("Init ?")
    dxl_io.set_goal_position(dict(zip(ids,angles)))
    raw_input("Start ?")
    rospy.Subscriber('/pynamixel/actuation', Actuation, actuate)
    rospy.spin()

if __name__ == '__main__': 
    port = ports.list_ports()[0]
    dxl_io = Dxl_IO(baudrate=1000000, port=port)
    print("Connected to port: " + port)
    start()
