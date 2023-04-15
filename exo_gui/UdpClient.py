import socket
from ctypes import *
import numpy as np
from ExoDataStruct import *
from PyQt5.QtCore import QMutex    


class QtLock(QMutex):
    def __init__(self):
       super().__init__()
    def __enter__(self):
        self.lock()
    def __exit__(self,*args):
        self.unlock()

class UdpClient:
    ip_address='192.168.1.4'
    data_port = UDP_DATA_PORT
    cmd_port = UDP_CMD_PORT
    def __init__(self):
        
        self.udp_cmd_packet = UdpCmdPacket()
        self.callback_fun =None
        self.flag = False
        self.udp_cmd_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        self.udp_data_socket = socket.socket(family=socket.AF_INET,type=socket.SOCK_DGRAM)
        self.udp_data_socket.bind(("0.0.0.0",self.data_port)) #python required server listen to 0.0.0.0
        self.udp_data_socket.settimeout(0.01)
        self.lock = QtLock()
        self.disconnect_count=0 #if continuously timeout, disconnect (call disConCcallback)
    def Connect(self):
        
        self.disconnect_count=0
        # self.udp_cmd_socket.bind((self.ip_address,self.cmd_port))
        self.flag=True
        return self.flag
    def Disconnect(self):
        self.flag=False
        
        
        

    def SetCallBack(self,callback_fun):
        self.callback_fun = callback_fun
        # self.updateJoint = updateJointFun
        # self.updatePre = updatePreFun
        # self.updateTank = updateTankFun
        # self.disConCcallback = disConCallback
        # self.recBtnUpdate = recBtnUpdate
        # self.conCondUpdate = conCondUpdate
        # self.pwm_lcd_update = pwm_lcd_update
        # self.calib_window_update = calibWindowUpdate
        # self.fsm_update = fsm_update
        # self.torque_update = torque_update
    
    def SetIP_Port(self,address,tx_port,rx_port):
        self.ip_address=address
        self.tx_port=tx_port
        self.rx_port=rx_port

    def ReqData(self):
    #     # this function will periodically called by QTimer
        
        if self.flag:
            
            server_address=(self.ip_address,UDP_CMD_PORT)
            # self.lock.lock()
            with self.lock:
                send_bytes= self.udp_cmd_socket.sendto(self.udp_cmd_packet,server_address)
            
            # self.lock.unlock()
            #reset all cmd flags after the current one is sent
            # pwm_flag_reset = [False]*PWM_VAL_NUM
            # reset_enc_flag_reset = [False]*NUM_ENC
            # des_pre_flag_reset = [False]*NUM_CHAMBER
            # des_imp_flag_reset = [False]*NUM_JOINT
            # des_force_flag_reset = [False]*NUM_JOINT
            # con_on_off_flag_reset = [False]*

            
            #   ("reset_enc_flag",(c_bool)*NUM_ENC),
            #   ("des_pre_flag",(c_bool)*NUM_CHAMBER),
            #   ("des_imp_flag",(c_bool)*NUM_JOINT),
            #   ("des_force_flag",(c_bool)*NUM_JOINT),
            #   ("epoch_time_flag",c_bool),
            #   ("recorder_flag",c_bool),
            #   ("con_on_off_flag",(c_bool)*NUM_JOINT)

            # self.udp_cmd_packet.pwm_duty_flag = (ctypes.c_bool * PWM_VAL_NUM)(*pwm_flag_reset)
            self.udp_cmd_packet = UdpCmdPacket()
            self.CheckRecv()
            
            
            
            


    def CheckRecv(self):
        
        # this function will be periodically called by QTimer
        try:
            data_recv = self.udp_data_socket.recvfrom(ctypes.sizeof(UdpDataPacket))
            
            if(len(data_recv[0])==ctypes.sizeof(UdpDataPacket)):
                self.disconnect_count=0
                udp_data_packet = UdpDataPacket.from_buffer_copy(data_recv[0])
                self.callback_fun['joint_plot_update'](udp_data_packet.enc_data)
                self.callback_fun['pressure_plot_update'](udp_data_packet.pre_data1)
                self.callback_fun['torque_update'](udp_data_packet.enc_data,udp_data_packet.pre_data1)
                self.callback_fun['air_volume_update'](udp_data_packet.pre_data1[TANK_ADC])
                

                self.callback_fun['rec_btn_update'](udp_data_packet.rec_status)
                self.callback_fun['pwm_duty_update'](udp_data_packet.pwm_duty)
                self.callback_fun['fsm_state_update'](udp_data_packet.fsm_state)
                self.callback_fun['cylncalib_update'](udp_data_packet.enc_data)
                
                self.callback_fun['phase_plot_update'](udp_data_packet.enc_data)
                
                
                
                
                
        except:
            self.disconnect_count = self.disconnect_count+1
            if(self.disconnect_count>100):
                # self.disConCcallback()
                self.callback_fun['disconnect']()






    