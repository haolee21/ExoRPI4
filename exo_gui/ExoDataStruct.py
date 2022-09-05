from ctypes import *


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

#adc index
LTANK_ADC = 0
LKNE_EXT_ADC=1
FORCE_ADC =2
POS_ADC=3
TANK_ADC=4
LKNE_FLEX_ADC=5



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