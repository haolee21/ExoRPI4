import socket 
import multiprocessing as mp
import struct
import threading as th
import time
import numpy as np
from PyQt5 import QtCore
import pdb
from threading import Lock
CMD_LEN = 2
ENC_NUM = 6
PRE_NUM = 8
DOUBLE_SIZE = 8
JOINT_DATA_LEN=ENC_NUM*DOUBLE_SIZE
PRE_DATA_LEN= PRE_NUM*DOUBLE_SIZE
MPC_LEN=4
class TCP:
    port =1234
    ip_address='192.168.0.104'
    def __init__(self):
        self.flag = False
        self.lock = Lock()
        
        

        
    def SetIP_Port(self,address,port):
        self.ip_address = address
        self.port = port
    
    def Connect(self):
        self.flag=True
        # try:
        #     self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #     self.s.connect((self.ip_address,self.port))
        #     self.flag=True
        #     print('server connected')
        # except:
        #     print('server refused')
       
        
        
        

        # except:
        #     print('sock connecting failed')
        return self.flag
    def SetCallBack(self,updateJointFun,updatePreFun,updateTankFun,disConCallback,recBtnUpdate,mpcCondUpdate):
        self.updateJoint = updateJointFun
        self.updatePre = updatePreFun
        self.updateTank = updateTankFun
        self.disConCcallback = disConCallback
        self.recBtnUpdate = recBtnUpdate
        self.mpcCondUpdate = mpcCondUpdate
        
    def Disconnect(self):
        if self.flag:
            self.flag=False
            
    def MP_test(self):
        print('this is the test function for testing multiprocess')
    def DataUpdate(self):
        if self.flag:
            receive = self.SendCmd('REQ:MEAS:DATA',JOINT_DATA_LEN+PRE_DATA_LEN,print_response=False)
            # receive =b''
            # # while True:
            # #     cur_data = self.s.recv(1)
            # #     if cur_data == b'\n':
            # #         break
            # #     else:
            # #         receive = receive+cur_data
            # receive = self.s.recv(JOINT_DATA_LEN+PRE_DATA_LEN+1)
            # receive = receive[:-1]
            # if len(receive) !=(JOINT_DATA_LEN+PRE_DATA_LEN):
            #     print('package incomplete')
            #     print(len(receive))
            # else:

            self.updateJoint(receive[:JOINT_DATA_LEN]) 
            self.updatePre(receive[JOINT_DATA_LEN:JOINT_DATA_LEN+PRE_DATA_LEN])
            self.updateTank(struct.unpack("d",receive[JOINT_DATA_LEN+4*DOUBLE_SIZE:JOINT_DATA_LEN+5*DOUBLE_SIZE])[0])
            # self.updateTank(int.from_bytes(receive[JOINT_DATA_LEN+4*DOUBLE_SIZE:JOINT_DATA_LEN+5*DOUBLE_SIZE],'little'))

            # rec_flag = int.from_bytes(self.SendCmd('REQ:REC:DATA',1,print_response=False),'little')
            rec_flag = int(self.SendCmd('REQ:REC:DATA',1,print_response=False).decode())
            # continuous checking if rec has already started

            self.recBtnUpdate(rec_flag)

            mpc_flag = self.SendCmd('REQ:CONT:MPC',MPC_LEN,print_response=False)
            # # print('mpc recv flag: ',(mpc_flag[0:2] and [False,True]))
            self.mpcCondUpdate(mpc_flag)
            

            
        else:
            print('error: tcp port not connected')
            self.disConCcallback()
            
    def SendCmd(self,cmd,byte_to_read,print_response=True):
        with self.lock:
            cmd = cmd+'\n'
            response = b''
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
                
                    s.connect((self.ip_address,self.port))
                
                    s.sendall(cmd.encode())

                    # s.settimeout(0.1)
                    response = s.recv(byte_to_read)
                
                
                
                # response = response[:-1]
                

            except:
                print('TCP:Failed\n')
                self.flag=False
            
            


        # cmd = cmd+'\n'
        # try:
        #     self.s.send(cmd.encode('utf-8'))

        # except:
        #     print('tcp failed to send command: ',cmd)
        # response = self.s.recv(byte_to_read+1)#include \n
    
        # response = response[:-1]
            if print_response:
                print('Send: '+cmd[:-1].ljust(25)+'Response: '+response.decode())
        return response

    def Update_test(self):
        #this is for debug usage
        # QtCore.pyqtRemoveInputHook()
        print(self.port)
        # pdb.set_trace()
        receiveNum = (np.random.randint(100,size=int((JOINT_DATA_LEN+PRE_DATA_LEN)/2))).tolist()
        
        receive = b''
        for c in receiveNum:
            receive = receive+c.to_bytes(2,'big')
        # pdb.set_trace()
        self.updateJoint(receive[:JOINT_DATA_LEN]) 
        self.updatePre(receive[JOINT_DATA_LEN:-2])

def TextToFloat(text):
    try:
        res = float(text)
    except:
        res=0
    return res
        
