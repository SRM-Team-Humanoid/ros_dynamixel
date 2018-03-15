#!/usr/bin/env python
import rospy
from pynamixel.msg import Actuation
import numpy as np

def talker():
    pub = rospy.Publisher('actuation', Actuation, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Actuation()
	ids = range(1,19)
	msg.ids = ids
        ang = np.linspace(-20, 20, 100)
        msg.speeds = [1023 for id in ids]
        msg.angles = [0.0 for id in ids]
        for i in ang:
	    msg.angles = [i for id in ids]
	    msg.speeds = [1023 for id in ids]
            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
