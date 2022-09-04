import ctypes
import socket
from ctypes import *
import numpy as np

# from struct import Struct
UDP_CMD_PORT=25000
UDP_DATA_PORT=25001
PWM_VAL_NUM=16
NUM_ENC = 6
NUM_PRE = 8
NUM_JOINT=4
NUM_CHAMBER=10

#encoder index
ENC_LHIP_S=0
ENC_LKNE_S=1
ENC_LANK_S=2
ENC_RHIP_S=3
ENC_RKNE_S=4
ENC_RANK_S=5

#joint controller
JOINT_LKNE=0
JOINT_LANK=1
JOINT_RKNE=2
JOINT_RANK=3

#chamber 
LKNE_EXT=0
LKNE_FLEX=1
LANK_EXT=2
LANK_FLEX=3
LTANK=4
RKNE_EXT=5
RKNE_FLEX=6
RANK_EXT=7
RANK_FLEX=8
RTANK=9

class UdpDataPacket(Structure):
    _fields_=[("pwm_duty",(c_byte)*PWM_VAL_NUM),
              ("enc_data",(c_double)*NUM_ENC),
              ("pre_data1",(c_double)*NUM_PRE),
              ("con_status",(c_bool)*NUM_JOINT),
              ("rec_status",c_bool)]
class UdpCmdPacket(Structure):
    _fields_=[("pwm_duty_data",(c_byte)*PWM_VAL_NUM),
              ("des_pre_data",(c_double)*NUM_CHAMBER),
              ("des_imp_data",(c_double)*NUM_JOINT),
              ("des_force_data",(c_double)*NUM_JOINT),
              ("epoch_time_data",c_double),
              ("recorder_data",c_bool),
              ("con_on_off_data",(c_bool)*NUM_JOINT),
              ("pwm_duty_flag",(c_bool)*PWM_VAL_NUM),
              ("reset_enc_flag",(c_bool)*NUM_ENC),
              ("des_pre_flag",(c_bool)*NUM_CHAMBER),
              ("des_imp_flag",(c_bool)*NUM_JOINT),
              ("des_force_flag",(c_bool)*NUM_JOINT),
              ("epoch_time_flag",c_bool),
              ("recorder_flag",c_bool),
              ("con_on_off_flag",(c_bool)*NUM_JOINT)]
    
class UdpClient:
    ip_address='192.168.0.104'
    data_port = UDP_DATA_PORT
    cmd_port = UDP_CMD_PORT
    def __init__(self):
        
        self.udp_cmd_packet = UdpCmdPacket()
        self.flag = False
        self.udp_cmd_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        self.udp_data_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        self.udp_data_socket.bind(("0.0.0.0",self.data_port)) #python required server listen to 0.0.0.0
       
    def Connect(self):
        
        
        # self.udp_cmd_socket.bind((self.ip_address,self.cmd_port))
        self.flag=True
        return self.flag
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
            
            server_address=(self.ip_address,UDP_CMD_PORT)
            send_bytes= self.udp_cmd_socket.sendto(self.udp_cmd_packet,server_address)
            
            #reset all cmd flags after the current one is sent
            pwm_flag_reset = [False]*PWM_VAL_NUM
            self.udp_cmd_packet.pwm_duty_flag = (ctypes.c_bool * PWM_VAL_NUM)(*pwm_flag_reset)

            
            
            
            self.CheckRecv()
            


    def CheckRecv(self):
        # this function will be periodically called by QTimer
        data_recv = self.udp_data_socket.recvfrom(ctypes.sizeof(UdpDataPacket))
        udp_data_packet = UdpDataPacket.from_buffer_copy(data_recv[0])

        
        self.updateJoint(udp_data_packet.enc_data)
        self.updatePre(udp_data_packet.pre_data1)
        self.updateTank(udp_data_packet.pre_data1[4]) 
        self.recBtnUpdate(udp_data_packet.rec_status)
        

def TextToFloat(text):
    try:
        res = float(text)
    except:
        res=0
    return res
def TextToInt(text):
    try:
        res = int(text)
    except:
        res=0
    return res




    