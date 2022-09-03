import ctypes
import socket
from ctypes import *

# from struct import Struct
UDP_CMD_PORT=25000
UDP_DATA_PORT=25001
PWM_VAL_NUM=16
NUM_ENC = 6
NUM_PRE = 8
NUM_JOINT=4
NUM_CHAMBER=10

class UdpDataPacket(Structure):
    _fields_=[("pwm_duty",(c_bool)*PWM_VAL_NUM),
              ("enc_data",(c_double)*NUM_ENC),
              ("pre_data1",(c_double)*NUM_PRE),
              ("con_status",(c_bool)*NUM_JOINT)]
class UdpCmdPacket(Structure):
    _fields_=[("pwm_duty_data",(c_byte)*PWM_VAL_NUM),
              ("des_pre_data",(c_double)*NUM_CHAMBER),
              ("des_imp_data",(c_double)*NUM_JOINT),
              ("des_force_data",(c_double)*NUM_JOINT),
              ("pwm_duty_flag",(c_bool)*PWM_VAL_NUM),
              ("reset_enc_flag",(c_bool)*NUM_ENC),
              ("des_pre_flag",(c_bool)*NUM_CHAMBER),
              ("des_imp_flag",(c_bool)*NUM_JOINT),
              ("des_force_flag",(c_bool)*NUM_JOINT)]
class UdpClient:
    ip_address="192.168.0.104"
    data_port = UDP_DATA_PORT
    cmd_port = UDP_CMD_PORT
    def __init__(self):
        self.udp_data_packet = UdpDataPacket()
        self.udp_cmd_packet = UdpCmdPacket()
        self.flag = False
        self.udp_cmd_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        self.udp_data_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        
        
    def Connect(self):
        self.udp_data_socket.bind((self.ip_address,self.data_port))
        self.udp_cmd_socket.bind((self.ip_address,self.cmd_port))
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
            self.udp_cmd_socket.sendall(self.udp_cmd_packet)
            
            


    def CheckRecv(self):
        # this function will be periodically called by QTimer
        data_recv = self.udp_data_socket.recvfrom(ctypes.sizeof(UdpDataPacket))
        udp_data_packet = UdpDataPacket.from_buffer_copy(data_recv[0])

        print(udp_data_packet.pre_data1)






    