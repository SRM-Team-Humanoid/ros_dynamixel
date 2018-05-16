#!/usr/bin/env python
import rospy
from rminus3.msg import Actuation
import numpy as np
from pynamixel.protocol1_0 import IO
from pynamixel.ports import *

def talker():
    pub = rospy.Publisher('actuation', Actuation, queue_size=10)
    rospy.init_node('talker', anonymous=False)
    rospy.set_param('debug', False)
    rate = rospy.Rate(10) # 10hz
    #rospy.loginfo(rospy.get_param('/talker/debug'))
    dxl_io = IO(port='/dev/ttyUSB1', baudrate=1000000)
    while not rospy.is_shutdown():
        #msg = Actuation()
        #ids = range(1,5)
        #msg.ids = ids
        #ang = np.linspace(-20, 20, 100)
        #msg.speeds = [1023 for id in ids]
        #msg.angles = [0.0 for id in ids]
        print(dxl_io.get_fsr(foot='left'))
        #for i in ang:
        #    msg.angles = [i for id in ids]
        #    msg.speeds = [1023 for id in ids]
        #    #rospy.loginfo(msg)
        #    pub.publish(msg)
        #    rate.sleep()
        pass

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
