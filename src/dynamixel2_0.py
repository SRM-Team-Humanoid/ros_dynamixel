#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, ctypes
from registers import *
from ports import *
from constants import *

os.sys.path.append('./dynamixel_functions_py')             # Path setting

import dynamixel_functions as dxl

class Dynamixel(object):
    def __init__(self, baudrate = 1000000, port = '/dev/ttyUSB0', protocol = 2):
        self.baudrate = baudrate
        self.port = dxl.portHandler(port.encode('utf-8'))
        self.protocol = protocol
        self.resolution = MX_RESOLUTION# MX-28 Resolution
        self.groupwrite = dxl.groupSyncWrite(self.port, self.protocol, ADDR_GOAL_POS, LEN_GOAL_POSITION)
        self.groupread = dxl.groupSyncRead(self.port, self.protocol, ADDR_PRES_POS, LEN_PRESENT_POSITION)
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
            print(dxl.getRxPacketError(self.protocol, error))

    def check_result(self):
        comm_result = dxl.getLastTxRxResult(self.port, self.protocol)
        if comm_result != COMM_SUCCESS:
            print(dxl.getTxRxResult(self.protocol, comm_result))

    # Set dxl operation mode (1: Vel control, 3: Position Control, 4: Multi-turn, 16: PWM)
    def set_op_mode(self, ids, mode = 3):
        for id in ids:
            dxl.write1ByteTxRx(self.port, self.protocol, id, ADDR_OP_MODE, mode)
            self.check_result()
            self.check_error()

    def set_torque_status(self, ids, value):
        for id in ids:
            dxl.write1ByteTxRx(self.port, self.protocol, id, ADDR_TORQUE_ENABLE, value)
            self.check_result()
            self.check_error()

    def set_profile_velocity(self, ids, value):
        for id in ids:
            dxl.write4ByteTxRx(self.port, self.protocol, id, ADDR_PROF_VEL, value)
            self.check_result()
            self.check_error()

    def set_acc_profile(self, ids, value):
        for id in ids:
            dxl.write4ByteTxRx(self.port, self.protocol, id, ADDR_PROF_ACC, value)
            self.check_result()
            self.check_error()

    def to_degree(self, value):
        angle = int(value*self.resolution)
        return angle

    def from_degree(self, angle):
        degree = int(float(angle)/self.resolution)
        return degree

    def write(self, write_dict):
        goal_position = 0
        for id,angle in write_dict.items():
            # Add parameter storage for dxl present position value
            addparam_result = ctypes.c_ubyte(dxl.groupSyncReadAddParam(self.groupread, id)).value
            if addparam_result != 1:
                print("[ID:%03d] groupSyncRead addparam failed" % (id))
                quit()

        for id,angle in write_dict.items():
            angle = self.from_degree(angle)
            print(angle)
            self.set_torque_status([id], value=1)

            diff = abs(angle - self.read_angle([id])[0])
            if diff > goal_position:
                goal_position = diff
                max_id = id

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


        while True:
            # Syncread present position
            dxl.groupSyncReadTxRxPacket(self.groupread)
            self.check_result()

            # Check if groupsyncread data of dxl#1 is available
            for id in write_dict.keys():
                getdata_result = ctypes.c_ubyte(dxl.groupSyncReadIsAvailable(self.groupread, id, ADDR_PRES_POS,LEN_PRESENT_POSITION)).value
                if getdata_result != 1:
                    print("[ID:%03d] groupSyncRead getdata failed" % (id))
                    quit()

            # Get dxl present position value
            present_position = dxl.groupSyncReadGetData(self.groupread, max_id, ADDR_PRES_POS, LEN_PRESENT_POSITION)

            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (max_id, goal_position, present_position))

            if not ((abs(goal_position - present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

    def read_angle(self, ids):
        positions = []
        for id in ids:
            present_position = dxl.read4ByteTxRx(self.port, self.protocol , id, ADDR_PRES_POS)
            self.check_result()
            self.check_error()
            present_position = self.to_degree(present_position)
            positions.append(present_position)
        return positions

    def set_secondary_id(self, ids, secid):
        for id in ids:
            dxl.write1ByteTxRx(self.port, self.protocol, id, ADDR_SECONDARY_ID, secid)
            self.check_result()
            self.check_error()
