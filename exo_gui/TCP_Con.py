import socket 
import multiprocessing as mp
import threading as th
import time
import numpy as np
from PyQt5 import QtCore
import pdb
CMD_LEN = 2
JOINT_DATA_LEN=12
PRE_DATA_LEN=10
class TCP:
    port =1234
    ip_address='192.168.0.100'
    def __init__(self):
        self.flag = False
        
        
        

        
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
    def SetCallBack(self,updateJointFun,updatePreFun,updateTankFun,disConCallback):
        self.updateJoint = updateJointFun
        self.updatePre = updatePreFun
        self.updateTank = updateTankFun
        self.disConCcallback = disConCallback
        
    def Disconnect(self):
        if self.flag:
            self.flag=False
            
    def MP_test(self):
        print('this is the test function for testing multiprocess')
    def DataUpdate(self):
        if self.flag:
            receive = self.SendCmd('REQ:MEAS:DATA',JOINT_DATA_LEN+PRE_DATA_LEN)
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
            self.updatePre(receive[JOINT_DATA_LEN:-2])
            self.updateTank(int.from_bytes(receive[-2:],'little'))
        else:
            print('error: tcp port not connected')
            self.disConCcallback()
            
    def SendCmd(self,cmd,byte_to_read):
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
        
