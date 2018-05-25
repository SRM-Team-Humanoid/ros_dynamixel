#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, ctypes
from reg1_0 import *
from ports import *
from constants import *

#os.sys.path.append('./dynamixel_functions_py')             # Path setting

import dynamixel_functions as dxl


#Daisy chain IO class for protocol 1
class Dxl_IO(object):
    def __init__(self, baudrate = 1000000, port = '/dev/ttyUSB0', protocol = 1):
        self.baudrate = baudrate
        self.port = dxl.portHandler(port.encode('utf-8'))
        self.protocol = protocol
        self.mx_res = MX_RESOLUTION # MX-28 Resolution
        self.fsr_res = FSR_RESOLUTION #FSR Resolution
        self.groupwrite = dxl.groupSyncWrite(self.port, self.protocol, ADDR_GOAL_POS, LEN_GOAL_POSITION)
        # self.groupspeed = dxl.groupSyncWrite(self.port, self.protocol, ADDR_MOV_SPEED, LEN_MOV_SPEED)
        # self.groupread = dxl.groupSyncRead(self.port, self.protocol, ADDR_PRES_POS, LEN_PRESENT_POSITION)
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
    def check_error(self, id):
        error = dxl.getLastRxPacketError(self.port, self.protocol)
        if error != 0:
            print("[id: %d, %s]" %(id, dxl.getRxPacketError(self.protocol, error)))
            return False
        return True


    def check_result(self, id):
        comm_result = dxl.getLastTxRxResult(self.port, self.protocol)
        if comm_result != COMM_SUCCESS:
            print("[id: %d, %s]" %(id, dxl.getTxRxResult(self.protocol, comm_result)))
            return False
        return True


    def set_torque_status(self, ids, value):
        for id in ids:
            dxl.write1ByteTxRx(self.port, self.protocol, id, ADDR_TORQUE_ENABLE, value)
            self.check_result(id)
            self.check_error(id)

    def to_degree(self, value):
        angle = int(value*self.mx_res)
        return angle

    def from_degree(self, angle):
        degree = int(float(angle)/self.mx_res)
        return degree

    def set_moving_speed(self, vel_dict):
        for id, vel in vel_dict.items():
            dxl.write2ByteTxRx(self.port, self.protocol, id, vel,  ADDR_MOV_SPEED)
            self.check_result(id)
            self.check_error(id)

    def is_moving(self,ids):
        mov = {}
        for id in ids:
            val = dxl.read1ByteRx(self.port, self.protocol, id, ADDR_MOVING)
            mov[id] = val
        return mov

    def set_goal_position(self, write_dict):
        #self.set_torque_status(write_dict.keys(),1)
        for id,angle in write_dict.items():
            angle = self.from_degree(angle+180)
            # Add dxl goal position value to the Syncwrite storage
            addparam_result = ctypes.c_ubyte(dxl.groupSyncWriteAddParam(self.groupwrite, id, angle, LEN_GOAL_POSITION)).value
            if addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (id))
                quit()

        # Syncwrite goal position
        dxl.groupSyncWriteTxPacket(self.groupwrite)
        self.check_result(id)

        # Clear syncwrite parameter storage
        dxl.groupSyncWriteClearParam(self.groupwrite)
        self.check_result(id)
        
    def get_present_position(self, ids):
        positions = []
        for id in ids:
            present_position = dxl.read2ByteTxRx(self.port, self.protocol, id, ADDR_PRES_POS)
            # if not self.check_result() and not self.check_error():
            present_position = self.to_degree(present_position)-180
            positions.append(present_position)
        return dict(zip(ids,positions))
    
    def to_newton(self,value):
        newton = float(value)*self.fsr_res
        return newton    

    def get_fsr_readings(self, foot):
        id_dic = {'left':111,'right':112}
        id = id_dic[foot]
        #fsr_reading = {'1':0, '2':0, '3':0, '4':0, 'x':0, 'y':0}
        fsr_reading = {}
        fsr_reg1 = {'1':ADDR_FSR_1, '2':ADDR_FSR_2, '3':ADDR_FSR_3, '4':ADDR_FSR_4}
        fsr_reg2 = {'x':ADDR_FSR_X, 'y':ADDR_FSR_Y}
        
        for reg in fsr_reg1.keys():
            fsr_reading[reg] = self.to_newton(dxl.read2ByteTxRx(self.port, self.protocol, id, fsr_reg1[reg]))
            self.check_result(id)
            self.check_error(id)
        for reg in fsr_reg2.keys():
            fsr_reading[reg] = dxl.read1ByteTxRx(self.port, self.protocol, id, fsr_reg2[reg])
            self.check_result(id)
            self.check_error(id)
        
        return fsr_reading 
    

