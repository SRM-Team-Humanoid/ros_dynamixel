#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pynamixel.protocol1_0 import Dxl_IO
from pynamixel.msg import FSR
import pynamixel.ports as ports

def publish_fsr():
    pub_left = rospy.Publisher('/pynamixel/fsr/left', FSR, queue_size=10)
    pub_right = rospy.Publisher('/pynamixel/fsr/right', FSR, queue_size=10)
    rospy.init_node('fsr_reader', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    port = ports.list_port()[0]
    dxl_io = Dxl_IO(port=port, baudrate=1000000)
    while not rospy.is_shutdown():
        fsr_data = dxl_io.get_fsr_readings(foot='left')
        fsr = FSR()
        fsr.fsr1 = fsr_data['1']
        fsr.fsr2 = fsr_data['2']
        fsr.fsr3 = fsr_data['3']
        fsr.fsr4 = fsr_data['4']
        fsr.x = fsr_data['x']
        fsr.y = fsr_data['y']
        pub_left.publish(fsr)
        fsr_data = dxl_io.get_fsr_readings(foot='right')
        fsr = FSR()
        fsr.fsr1 = fsr_data['1']
        fsr.fsr2 = fsr_data['2']
        fsr.fsr3 = fsr_data['3']
        fsr.fsr4 = fsr_data['4']
        fsr.x = fsr_data['x']
        fsr.y = fsr_data['y']
        pub_right.publish(fsr)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fsr()
    except rospy.ROSInterruptException:
        pass
