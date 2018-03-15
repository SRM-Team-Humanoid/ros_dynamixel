#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, ctypes
from reg1_0 import *
from ports import *
from constants import *

import dynamixel_functions as dxl

class Dynamixel(object):
    def __init__(self, baudrate = 1000000, port = '/dev/ttyUSB0', protocol = 1):
        self.baudrate = baudrate
        self.port = dxl.portHandler(port.encode('utf-8'))
        self.protocol = protocol
        self.resolution = MX_RESOLUTION# MX-28 Resolution
        self.groupwrite = dxl.groupSyncWrite(self.port, self.protocol, ADDR_GOAL_POS, LEN_GOAL_POSITION)
        self.groupspeed = dxl.groupSyncWrite(self.port, self.protocol, ADDR_MOV_SPEED, LEN_MOV_SPEED)
        dxl.packetHandler()
        self.connect()

    def connect(self):
        if dxl.openPort(self.port):
            dxl.setBaudRate(self.port, self.baudrate)
        else:
            print("Failed to open the port!")
            quit()

    def disconnect(self):
        dxl.closePort(self.port)

    # Check for errors in the last comm
    def check_error(self):
        error = dxl.getLastRxPacketError(self.port, self.protocol)
        if error != 0:
            # print(dxl.getRxPacketError(self.protocol, error))
            return False
	else: return True


    def check_result(self):
        comm_result = dxl.getLastTxRxResult(self.port, self.protocol)
        if comm_result != COMM_SUCCESS:
            # print(dxl.getTxRxResult(self.protocol, comm_result))
            return False
	else: return True


    def set_torque_status(self, ids, value):
        for id in ids:
            dxl.write1ByteTxRx(self.port, self.protocol, id, ADDR_TORQUE_ENABLE, value)
            self.check_result()
            self.check_error()

    def to_degree(self, value):
        angle = int(value*self.resolution)
        return angle

    def from_degree(self, angle):
        degree = int(float(angle)/self.resolution)
        return degree

    def set_moving_speed(self, vel_dict):
	for id,vel in vel_dict.items():  
            # Add dxl velocity value to the Syncwrite storage
            addparam_result = ctypes.c_ubyte(dxl.groupSyncWriteAddParam(self.groupspeed, id, vel, LEN_MOV_SPEED)).value
            if addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (id))
                quit()

        # Syncwrite goal position
        dxl.groupSyncWriteTxPacket(self.groupspeed)
        self.check_result()

        # Clear syncwrite parameter storage
        dxl.groupSyncWriteClearParam(self.groupspeed)
        self.check_result()

    def is_moving(self,ids):
        mov = {}
        for id in ids:
            val = dxl.read1ByteRx(self.port, self.protocol, id, ADDR_MOVING)
            mov[id] = val
        return mov

    def write(self, write_dict):
        self.set_torque_status(write_dict.keys(),1)
	# goal_position = 0
        for id,angle in write_dict.items():
            angle = self.from_degree(angle+180)
            # print(angle)
            self.set_torque_status([id], value=1)

            # Add dxl goal position value to the Syncwrite storage
            addparam_result = ctypes.c_ubyte(dxl.groupSyncWriteAddParam(self.groupwrite, id, angle, LEN_GOAL_POSITION)).value
            if addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (id))
                quit()

        # Syncwrite goal position
        dxl.groupSyncWriteTxPacket(self.groupwrite)
        self.check_result()

        # Clear syncwrite parameter storage
        dxl.groupSyncWriteClearParam(self.groupwrite)
        self.check_result()

    def read(self, ids):
        positions = []
        for id in ids:
            present_position = dxl.read2ByteTxRx(self.port, self.protocol, id, ADDR_PRES_POS)
            # if not self.check_result() and not self.check_error():
            present_position = self.to_degree(present_position)-180
            positions.append(present_position)
        return dict(zip(ids,positions))

