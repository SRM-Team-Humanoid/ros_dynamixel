from dynamixel1_0 import Dynamixel
import time
from ports import *

port = list_port()[0]
print(port)
ids = range(1,7)
angles = [0.0 for id in ids]
dxl_io = Dynamixel(baudrate=1000000, port=port, protocol=1)
dxl_io.connect()
#dxl_io.set_moving_speed(ids, 1023)
print(dxl_io.read_angle(ids))
dxl_io.write(dict(zip(ids,angles)))
time.sleep(1)
dxl_io.disconnect()
