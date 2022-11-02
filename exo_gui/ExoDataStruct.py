from ctypes import *
import ctypes


# from struct import Struct
UDP_CMD_PORT=25000
UDP_DATA_PORT=25001
PWM_VAL_NUM=16
NUM_ENC = 6
NUM_PRE = 8
NUM_JOINT=4
NUM_CHAMBER=9

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
LTANK=3
RKNE_EXT=4
RKNE_FLEX=5
RANK_EXT=6
RTANK=7
ATMOS=8 #exhaust

#adc index
LTANK_ADC = 0
LKNE_EXT_ADC=1
FORCE_ADC =2
POS_ADC=3
TANK_ADC=4
LKNE_FLEX_ADC=5
LANK_EXT_ADC=6

#PWM ID, should be the enum PWM_ID value
LTANK_PWM=0
LKNE_EXT_PWM=1
LKNE_FLEX_PWM=2
LKNE_ANK_PWM=3
LKNE_EXUT_PWM=4
LANK_EXT_PWM=5
LANK_EXUT_PWM=6

RTANK_PWM=8
RKNE_EXT_PWM=9
RKNE_FLEX_PWM=10
RKNE_ANK_PWM=11
RKNE_EXUT_PWM=12
RANK_EXT_PWM=13
RANK_EXUT_PWM=14


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
              ("init_force",(c_double)*NUM_JOINT),
              ("init_impact_imp",(c_double)*NUM_JOINT),
              ("restore_imp",(c_double)*NUM_JOINT),

              ("pwm_duty_flag",(c_bool)*PWM_VAL_NUM),
              ("reset_enc_flag",(c_bool)*NUM_ENC),
              ("des_pre_flag",(c_bool)*NUM_CHAMBER),
              ("des_imp_flag",(c_bool)*NUM_JOINT),
              ("des_force_flag",(c_bool)*NUM_JOINT),
              ("epoch_time_flag",c_bool),
              ("recorder_flag",c_bool),
              ("con_on_off_flag",(c_bool)*NUM_JOINT),
              ("set_joint_pos_flag",(c_bool)*NUM_JOINT),
              ("impact_absorb_flag",(c_bool)*NUM_JOINT),]
    def __init__(self):
        self.pwm_duty_flag=(ctypes.c_bool*PWM_VAL_NUM)((False)*PWM_VAL_NUM)
        self.reset_enc_flag = (ctypes.c_bool*NUM_ENC)((False)*NUM_ENC)
        self.des_pre_flag = (ctypes.c_bool*NUM_CHAMBER)((False)*NUM_CHAMBER)
        self.des_imp_flag = (ctypes.c_bool*NUM_JOINT)((False)*NUM_JOINT)
        self.des_force_flag = (ctypes.c_bool*NUM_JOINT)((False)*NUM_JOINT)
        self.epoch_time_flag = False
        self.recorder_flag=False
        self.con_on_off_flag = (ctypes.c_bool*NUM_JOINT)((False)*NUM_JOINT)
        self.set_joint_pos_flag = (ctypes.c_bool*NUM_JOINT)((False)*NUM_JOINT)