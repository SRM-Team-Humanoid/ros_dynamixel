from dynamixel1_0 import Dynamixel
import time
from ports import *
import numpy as np

port = list_port()[0]
print(port)
dxl_io = Dynamixel(baudrate=1000000, port=port, protocol=1)
ids = range(1,7)
dxl_io.set_moving_speed(dict(zip(ids, [1023 for id in ids])))
ang = [-20,20]
for a in ang:
    angles = [a for id in ids]
    #print(dxl_io.read_angle(ids))
    dxl_io.write(dict(zip(ids,angles)))
    #time.sleep(0.01)
    #print(dxl_io.read_angle(ids))
dxl_io.disconnect()
