from scipy.signal import butter,filtfilt
import numpy as np
import pdb
import pandas as pd
import glob
# Load the data and transform them to SI units
# low pass force and position to reduce noise
def DataProcess(fileName,data=None,static=False,filtered=True,startTime=None,endTime=None,force_bias=539.91825,max_pos=2.6346146336996337):
#     pdb.set_trace()
    if data is None:
        data=pd.read_csv(fileName)
    data=data.apply(lambda x:GetPos(x/4095*3.3,max_pos)/1000 if x.name=='pos' else x)
    data=data.apply(lambda x:(x/4095*3.3-0.5)/(4.5-0.5)*200*6894.76 if x.name=='cyln_pre' or x.name=='tank_pre' else x)
    amp = 1+50*1000/220
  
    data=data.apply(lambda x:(((x-force_bias)/4095*3.3)/amp)*2500/0.1 if x.name=='force' else x)
    
    if static:
        data = data.iloc[-500:,:]
    
    # low poss filter force and pos
    nyq = 0.5*1000 # from sampling rate
    cutoff = 3
    order =4
    normal_cutoff=cutoff/nyq
    b,a = butter(order,normal_cutoff,btype='low',analog=False)
    force_filt = filtfilt(b,a,data['force'])
    pos_filt = filtfilt(b,a,data['pos'])
    
    cyln_pre_filt =filtfilt(b,a,data['cyln_pre'])
    tank_pre_filt = filtfilt(b,a,data['tank_pre'])
    
    
    tank_pre_mean = np.mean(data['tank_pre'])
    # low pass filter pressure
    if filtered:
        data['pos']=pos_filt
        data['cyln_pre']=cyln_pre_filt
        data['force']=force_filt
        data['tank_pre']=tank_pre_filt
    
    
        data['dCyln_pre']=np.gradient(cyln_pre_filt)
        data['dTank_pre']=np.gradient(tank_pre_filt)

        data['dForce']=np.gradient(force_filt)
        data['dPos']=np.gradient(pos_filt)
        data['dVel']=np.gradient(data['dPos'])
    else:
        data['dCyln_pre']=np.gradient(data['cyln_pre'])
        data['dTank_pre']=np.gradient(data['tank_pre'])

        data['dForce']=np.gradient(data['force'])
        data['dPos']=np.gradient(data['pos'])
        data['dVel']=np.gradient(data['dPos'])
        
    
    if startTime != None:
        data=data[data['time']>startTime]
    if endTime !=None:
        data=data[data['time']<endTime]
    
        

    return data

pos=[2.27,3.94,7.54,11.34,14.01,17.01,19.15]
volt=np.array([0.7113,0.9618,1.2694,1.7673,1.9386,2.1732,2.4259])*3.3/5 # it was originally calibrate with 5V
beta = np.linalg.lstsq(np.vstack([np.ones([1,len(volt)]),volt]).T,pos,rcond=None)
L = 6*25.4 #unit mm
# max_pos_v = 2.3720 # no spring
# max_pos_v = 3.371  # spring

def _getPos(volt):
    return beta[0][0]+beta[0][1]*np.array(volt)

# pos_bias = L - _getPos(max_pos_v)


def GetPos(volt,max_pos_v):
    return beta[0][0]+beta[0][1]*np.array(volt)+L - _getPos(max_pos_v)

def LoadSameDuty(duty,startTime=None):
#     pdb.set_trace()
    data = pd.concat([DataProcess(file,startTime=startTime) for file in glob.glob('Static_flow/duty_'+str(duty)+"/*.csv")])
    data=data.drop(columns='time')
    return data
def AddDelay(duty,delay,startTime=None,endTime=None):
    res_data_highTank=[]
    res_data_highCyln=[]
    for file in glob.glob('Static_flow/duty_'+str(duty)+'/*.csv'):
        delayData=pd.DataFrame()
        data = DataProcess(file,startTime=startTime,endTime=endTime)
        
        delayData['cyln_pre'] = data.iloc[:-delay,data.columns.get_loc('cyln_pre')]
        delayData['tank_pre'] = data.iloc[:-delay,data.columns.get_loc('tank_pre')]
        delayData['dCyln_pre'] = np.array(data.iloc[delay:,data.columns.get_loc('dCyln_pre')])
        delayData['dTank_pre'] = np.array(data.iloc[delay:,data.columns.get_loc('dTank_pre')])  
#         pdb.set_trace()
        delayData['duty']=np.array([duty]*(len(data['cyln_pre'])-delay))
        if np.mean(data['cyln_pre'])>np.mean(data['tank_pre']):
            res_data_highCyln.append(delayData)
        else:
            res_data_highTank.append(delayData)

    return pd.concat(res_data_highCyln),pd.concat(res_data_highTank)
def DelaySingleData(file,duty,delay,startTime=None,endTime=None):
    delayData=pd.DataFrame()
    data = DataProcess(file,startTime=startTime,endTime=endTime)
    delayData['cyln_pre'] = data.iloc[:-delay,data.columns.get_loc('cyln_pre')]
    delayData['tank_pre'] = data.iloc[:-delay,data.columns.get_loc('tank_pre')]
    delayData['dCyln_pre'] = np.array(data.iloc[delay:,data.columns.get_loc('dCyln_pre')])
    delayData['dTank_pre'] = np.array(data.iloc[delay:,data.columns.get_loc('dTank_pre')])  
    delayData['duty']=np.array([duty]*(len(data['cyln_pre'])-delay))
#     pdb.set_trace()
    delayData['time']=data.iloc[:-delay,data.columns.get_loc('time')]
    if np.mean(data['cyln_pre'])>np.mean(data['tank_pre']):
        return delayData,[]
    else:
        return [],delayData
        
def DelayMultiData(file,delay,startTime=None,endTime=None):
    delayData=pd.DataFrame()
    data = DataProcess(file,startTime=startTime,endTime=endTime)
    delayData['cyln_pre'] = data.iloc[:-delay,data.columns.get_loc('cyln_pre')]
    delayData['tank_pre'] = data.iloc[:-delay,data.columns.get_loc('tank_pre')]
    delayData['dCyln_pre'] = np.array(data.iloc[delay:,data.columns.get_loc('dCyln_pre')])
    delayData['dTank_pre'] = np.array(data.iloc[delay:,data.columns.get_loc('dTank_pre')])  
    delayData['duty']=data.iloc[:-delay,data.columns.get_loc('duty')]
#     pdb.set_trace()
    delayData['time']=data.iloc[:-delay,data.columns.get_loc('time')]
    if np.mean(data['cyln_pre'])>np.mean(data['tank_pre']):
        return delayData,[]
    else:
        return [],delayData
        