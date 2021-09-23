import socket
import time
import numpy as np
import pdb
# HOST='192.168.1.194'
HOST = '127.0.0.1'
PORT=1234
JOINT_DATA_LEN=12
PRE_DATA_LEN=10
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    s.settimeout(0.01)
    with conn:
        print('Connected by', addr)
        old_cmd=b''
        while True:
            recv_cmd=b''
            while True:
                data = conn.recv(4)
                idx = data.find(b'\n')
                if idx<0:
                    old_cmd = old_cmd+data
                    
                else:
                    recv_cmd=old_cmd+data[:idx]
                    old_cmd = data[idx+1:]
                    break
            if recv_cmd == b'REQ:MEAS':
                receiveNum = (np.random.randint(100,size=int((JOINT_DATA_LEN+PRE_DATA_LEN)/2))).tolist()
                receive = b''
                for c in receiveNum:
                    receive = receive+c.to_bytes(2,'big')
                receive = receive+b'\n'
                print('send ',len(receive))
                conn.send(receive)
                # time.sleep(0.02)
            elif recv_cmd == b'CON:STOP':
                conn.send(b'OK\n')
                break
            else:
                print('client sent different msg')
                print(recv_cmd.decode())
                ret='OK\n'
                conn.send(ret.encode('utf-8'))
            


                
