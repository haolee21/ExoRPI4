import json
from ctypes import *
import os
MPC_STATE_NUM = 125

class PhyParam(dict):
    def __init__(self) -> None:
        # self.params = {'CylnMaxMechLen':0,'CylnChamberMaxLen':0,'FrictionCoeff':0,'PistonArea_mm2':[0,0],'SpringConst':0,'CylnEqn':[0,0,0],'NeutralPos':0}
        super().__init__()
        self['CylnMaxMechLen'] = 0
        self['CylnChamberMaxLen'] =0
        self['FrictionCoeff'] = 0
        self['PistonArea_mm2'] = [0,0]
        self['SpringConst'] = 0
        self['CylnEqn'] = [0,0,0] #cylinder length = coeff[0]-coeff[1]*(angle-coeff[2]) 
        self['NeutralPos'] = 0
        
    # def __getitem__(self,item):

class MPC_Param(dict):
    def __init__(self) -> None:
        super().__init__()
        self['ch'] = {'a':[0]*MPC_STATE_NUM,'b':[0]*MPC_STATE_NUM}
        self['cl'] = {'a':[0]*MPC_STATE_NUM,'b':[0]*MPC_STATE_NUM}
        

class ConfigLoader:

    def __init__(self) -> None:
        
        # self.phy_config_file_load = False
        # self.mpc_config_file_load=False
        self.phy_config_name = ''
        self.mpc_config_name = ''

        self.phy_config_path = os.getcwd()
        self.mpc_config_path = os.getcwd()

        # self.phy_config_file_modified = False
        # self.mpc_config_file_modified = False
        self.phy_param = {'LeftKnee':PhyParam(),'LeftAnkle':PhyParam(),'RightKnee':PhyParam(),'RightAnkle':PhyParam()}
        self.mpc_param = {'LTankSubtank':MPC_Param(),'LTankKne':MPC_Param(),'LTankRAnk':MPC_Param(),'LKneRAnk':MPC_Param(),
                          'RTankSubtank':MPC_Param(),'RTankKne':MPC_Param(),'RTankLAnk':MPC_Param(),'RKneLAnk':MPC_Param()}

        self.LoadPhyConfig('exo_config.json')
        self.LoadMPC_Config('mpc_config.json')
    def LoadSinglePhy(phy_json,phy_param):

        # phy_param = phy_json

        phy_param['CylnEqn'] = phy_json['CylnEqn']
        phy_param['PistonArea_mm2']= phy_json['PistonArea_mm2']
        phy_param['SpringConst'] = phy_json['SpringConst']
        phy_param['CylnChamberMaxLen'] = phy_json['CylnChamberMaxLen']
        phy_param['CylnMaxMechLen'] = phy_json['CylnMaxMechLen']
        phy_param['NeutralPos'] = phy_json['NeutralPos']
        phy_param['FrictionCoeff'] = phy_json['FrictionCoeff']

    def LoadSingleMPC(mpc_json,mpc_param):
        # mpc_param = mpc_json
        mpc_param['ch'] = mpc_json['ch']
        mpc_param['cl'] = mpc_json['cl']

    def LoadPhyConfig(self,file_name):
        with open(file_name) as json_file:
            phy_json = json.load(json_file)
            ConfigLoader.LoadSinglePhy(phy_json=phy_json['LeftKnee'],phy_param= self.phy_param['LeftKnee'])
            ConfigLoader.LoadSinglePhy(phy_json=phy_json['LeftAnkle'],phy_param= self.phy_param['LeftAnkle'])
            ConfigLoader.LoadSinglePhy(phy_json=phy_json['RightKnee'],phy_param= self.phy_param['RightKnee'])
            ConfigLoader.LoadSinglePhy(phy_json=phy_json['RightAnkle'],phy_param=self.phy_param['RightAnkle'])

            self.phy_config_file_load = True

            self.phy_config_path = os.path.dirname(file_name)
        return
    def LoadMPC_Config(self,file_name):
        with open(file_name) as json_file:
            mpc_json = json.load(json_file)
            ConfigLoader.LoadSingleMPC(mpc_json['LTankSubtank'],self.mpc_param['LTankSubtank'])
            ConfigLoader.LoadSingleMPC(mpc_json['LTankKne'],self.mpc_param['LTankKne'])
            ConfigLoader.LoadSingleMPC(mpc_json['LTankRAnk'],self.mpc_param['LTankRAnk'])
            # ConfigLoader.LoadSingleMPC(mpc_json['LKneeExtFlex'],self.mpc_param['LKneeExtFlex'])
            # ConfigLoader.LoadSingleMPC(mpc_json['LAnkExtFlex'],self.mpc_param['LAnkExtFlex'])
            ConfigLoader.LoadSingleMPC(mpc_json['LKneRAnk'],self.mpc_param['LKneRAnk'])

            ConfigLoader.LoadSingleMPC(mpc_json['RTankSubtank'],self.mpc_param['RTankSubtank'])
            ConfigLoader.LoadSingleMPC(mpc_json['RTankKne'],self.mpc_param['RTankKne'])
            ConfigLoader.LoadSingleMPC(mpc_json['RTankLAnk'],self.mpc_param['RTankLAnk'])
            # ConfigLoader.LoadSingleMPC(mpc_json['RKneeExtFlex'],self.mpc_param['RKneeExtFlex'])
            # ConfigLoader.LoadSingleMPC(mpc_json['RAnkExtFlex'],self.mpc_param['RAnkExtFlex'])
            ConfigLoader.LoadSingleMPC(mpc_json['RKneLAnk'],self.mpc_param['RKneLAnk'])

            self.mpc_config_file_load=True
            self.mpc_config_path = os.path.dirname(file_name)
            
        return

    def UpdatePhyParam(self,joint_name,param_name,param_val):
        
        self.phy_param[joint_name][param_name] = param_val
        self.phy_config_file_modified = True
    def UpdateMPC_Param(self,mpc_con_name,param_val):
        self.mpc_param[mpc_con_name] = param_val
        self.mpc_config_file_modified = True


    def SavePhyConfig(self,file_name):
        with open(file_name,'w') as file:
            json.dump(self.phy_param,file)
            self.phy_config_file_modified = False
    def SaveMPC_Config(self,file_name):
        with open(file_name,'w') as file:
            json.dump(self.mpc_param,file)
            self.mpc_config_file_modified=False
    def GetPhyConfigCond(self):
        if self.phy_config_file_load and self.phy_config_file_modified:
            return 1
        elif self.phy_config_file_load:
            return -1  #did not load phy config file yet
        else:
            return 0