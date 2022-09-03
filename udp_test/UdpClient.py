import ctypes
import socket
from ctypes import *
import numpy as np

# from struct import Struct
UDP_CMD_PORT=35000
UDP_DATA_PORT=35001
PWM_VAL_NUM=16
NUM_ENC = 6
NUM_PRE = 8
NUM_JOINT=4
NUM_CHAMBER=10

class UdpDataPacket(Structure):
    _fields_=[("pwm_duty",(c_byte)*PWM_VAL_NUM),
              ("enc_data",(c_double)*NUM_ENC),
              ("pre_data1",(c_double)*NUM_PRE),
              ("con_status",(c_bool)*NUM_JOINT)]
class UdpCmdPacket(Structure):
    _fields_=[("pwm_duty_data",(c_byte)*PWM_VAL_NUM),
              ("des_pre_data",(c_double)*NUM_CHAMBER),
              ("des_imp_data",(c_double)*NUM_JOINT),
              ("des_force_data",(c_double)*NUM_JOINT),
              ("epoch_time_data",c_double),
              ("recorder_data",c_bool),
              ("pwm_duty_flag",(c_bool)*PWM_VAL_NUM),
              ("reset_enc_flag",(c_bool)*NUM_ENC),
              ("des_pre_flag",(c_bool)*NUM_CHAMBER),
              ("des_imp_flag",(c_bool)*NUM_JOINT),
              ("des_force_flag",(c_bool)*NUM_JOINT),
              ("epoch_time_flag",c_bool),
              ("recorder_flag",c_bool)]
class UdpClient:
    ip_address="127.0.0.1"
    data_port = UDP_DATA_PORT
    cmd_port = UDP_CMD_PORT
    def __init__(self):
        self.udp_data_packet = UdpDataPacket()
        self.udp_cmd_packet = UdpCmdPacket()
        self.flag = False
        self.udp_cmd_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        self.udp_data_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        
        self.udp_data_socket.bind((self.ip_address,self.data_port))
        # self.udp_cmd_socket.bind((self.ip_address,self.cmd_port))
    def Connect(self):
        # self.udp_data_socket.bind((self.ip_address,self.data_port))
        # self.udp_cmd_socket.bind((self.ip_address,self.cmd_port))
        self.flag=True
    def Disconnect(self):
        self.flag=False
        

    def SetCallBack(self,updateJointFun,updatePreFun,updateTankFun,disConCallback,recBtnUpdate,conCondUpdate):
        self.updateJoint = updateJointFun
        self.updatePre = updatePreFun
        self.updateTank = updateTankFun
        self.disConCcallback = disConCallback
        self.recBtnUpdate = recBtnUpdate
        self.conCondUpdate = conCondUpdate
    
    def SetIP_Port(self,address,tx_port,rx_port):
        self.ip_address=address
        self.tx_port=tx_port
        self.rx_port=rx_port

    def ReqData(self):
    #     # this function will periodically called by QTimer
        if self.flag:
            print("data packet size")
            print(ctypes.sizeof(UdpCmdPacket))
            self.udp_cmd_packet.des_pre_flag[0]=True

            server_address=(self.ip_address,UDP_CMD_PORT)
            send_bytes= self.udp_cmd_socket.sendto(self.udp_cmd_packet,server_address)
            print("done sending")
            print(send_bytes)
            


    def CheckRecv(self):
        # this function will be periodically called by QTimer
        data_recv = self.udp_data_socket.recvfrom(ctypes.sizeof(UdpDataPacket))
        udp_data_packet = UdpDataPacket.from_buffer_copy(data_recv[0])
        






    