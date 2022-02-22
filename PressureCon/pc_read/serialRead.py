
import serial
from datetime import date,datetime
import time
import pandas as pd
baud = 115200
dev_port = '/dev/ttyACM0'




while True:
    data=input('Start next sampling cycle? (y/n)')
    if data=='y':
        print('Buffer cleared')

        ser=serial.Serial(port=dev_port,baudrate=baud,timeout=None,parity=serial.PARITY_ODD,bytesize=serial.EIGHTBITS,stopbits=serial.STOPBITS_ONE)
        ser.flushInput()
        ser.flushOutput()
        time.sleep(0.1)
        print(dev_port, 'cleared, please press the reset button on arduino')
        
        # pc will receive data until '\n' is received
        msg=''
        data_list=[]
        while True:
            cur_byte=ser.read()
            if cur_byte == '\t':
                data_list.append(msg)
                msg=''
            elif cur_byte=='\n':
                data_list.append(msg)
                break
            else:
                msg +=cur_byte
        print('we received data from ', len(data_list), ' different sources')


        
        ## save the data
        
    
        result_data=dict()
        for idx,value in enumerate(data_list):
            name = 'data'+str(idx)
            result_data['data'+str(idx)]=value

        data_frame = pd.DataFrame(result_data)
        data_frame.to_csv(datetime.now().strftime("%H%M%S_%b%d%Y"))
        print('Done sampling')

    else:
        print('Invalid command')





