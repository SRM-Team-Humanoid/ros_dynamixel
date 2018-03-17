#!/usr/bin/env python
import rospy
from pynamixel.msg import Actuation
import numpy as np
#from pynamixel.dynamixel1_0 import Dynamixel
#from pynamixel.ports import *

def talker():
    pub = rospy.Publisher('actuation', Actuation, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #dxl_io = Dynamixel()
    while not rospy.is_shutdown():
        msg = Actuation()
        ids = range(1,5)
        msg.ids = ids
        ang = np.linspace(-20, 20, 100)
        msg.speeds = [1023 for id in ids]
        msg.angles = [0.0 for id in ids]
        for i in ang:
            msg.angles = [i for id in ids]
            msg.speeds = [1023 for id in ids]
            #dxl_io.set_moving_speed(dict(zip(ids,msg.speeds)))
            #dxl_io.write(dict(zip(ids,msg.angles)))
            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
