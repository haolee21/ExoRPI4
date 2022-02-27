
import serial
from datetime import date,datetime
import time
import pandas as pd
import pdb
baud = 100000
dev_port = '/dev/ttyACM0'




while True:
    data=input('Start next sampling cycle? (y/n)')
    if data=='y':
        
        input('Please press the reset button on arduino, hit enter after the red light went off')
        ser=serial.Serial(port=dev_port,baudrate=baud,timeout=None,parity=serial.PARITY_NONE,bytesize=serial.EIGHTBITS,stopbits=serial.STOPBITS_ONE,xonxoff=True)
        ser.flushInput()
        ser.flushOutput()
        
        
        # pc will receive data until '\n' is received
        msg=[]
        data_list=[]
        power=0
        val=0
        data_count=0
        while True:
            cur_byte=ser.read().decode('utf-8')
            if cur_byte==',':
                # print(val)
                msg.append(val)
                val=0
                pow=0
                data_count=data_count+1
            elif cur_byte == '\t':
                print('next data set')
                print('msg length ',len(msg))
                data_list.append(msg)
                data_count=0
                msg=[]
            elif cur_byte=='\n':
                
                print('end data sending')
                break
            else:
                val*=10
                val += int(cur_byte)
                # power +=1

        print('we received data from ', len(data_list), ' different sources')


        
        ## save the data
        
        # pdb.set_trace()
        data_label=['tank_pre','cyln_pre','force','pos','val1','val2']
        result_data=dict()
        for idx,value in enumerate(data_list):
            name = data_label[idx]
            
            result_data[name]=value

        data_frame = pd.DataFrame(result_data)
        data_frame.to_csv(datetime.now().strftime("%H%M%S_%b%d%Y.csv"),index_label='time')
        print('Done sampling')

        ser.close()

    else:
        print('Invalid command')





