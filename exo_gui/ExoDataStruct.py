from ctypes import *
import ctypes
from enum import Enum

# from struct import Struct
UDP_CMD_PORT=25000
UDP_DATA_PORT=25001
PWM_VAL_NUM=16
NUM_ENC = 6
NUM_PRE = 16
# NUM_JOINT=4
# NUM_CHAMBER=9

# constant from Valves_hub.hpp
NUM_KNEE_ANK_PAIR=2
LKRA = 0
RKLA = 1
# constant from JointCon.hpp
NUM_PRE_CON = 5
NUM_FORCE_CON=2

PRE_CON_SUBTANK=0
PRE_CON_KNE_EXT=1
PRE_CON_ANK_PLANT=2
PRE_CON_KNE_FLEX=3
PRE_CON_ANK_DORSI=4

class Chamber(Enum):
    SubTank=0
    KneExt=1
    KneFlex=2
    AnkPla=3
    AnkDor=4
    MainTank=5



FORCE_CON_KNE_EXT=0
FORCE_CON_ANK_PLANT=1


#encoder index
ENC_LHIP_S=0
ENC_LKNE_S=1
ENC_LANK_S=2
ENC_RHIP_S=3
ENC_RKNE_S=4
ENC_RANK_S=5

#joint controller
# JOINT_LKNE=0
# JOINT_LANK=1
# JOINT_RKNE=2
# JOINT_RANK=3

#chamber 
# LKNE_EXT=0
# LKNE_FLEX=1
# LANK_EXT=2
# LTANK=3
# RKNE_EXT=4
# RKNE_FLEX=5
# RANK_EXT=6
# RTANK=7
# ATMOS=8 #exhaust

#adc index
PRE_SEN1=0
PRE_SEN2=1
PRE_SEN3=2
PRE_SEN4=3
PRE_SEN5=4
PRE_SEN6=5
PRE_SEN7=6
PRE_SEN8=7
PRE_SEN9=8
PRE_SEN10=9
PRE_SEN11=10
PRE_SEN12=11
PRE_SEN13=12
PRE_SEN14=13
PRE_SEN15=14
PRE_SEN16=15





TANK_ADC=PRE_SEN14
RTANK_ADC=PRE_SEN9
LTANK_ADC = PRE_SEN12
LKNE_FLEX_ADC=PRE_SEN10
RKNE_FLEX_ADC=PRE_SEN11
RKNE_EXT_ADC=PRE_SEN16
RANK_EXT_ADC=PRE_SEN15
LKNE_EXT_ADC=PRE_SEN2
LANK_EXT_ADC=PRE_SEN1
LANK_FLEX_ADC = PRE_SEN5
RANK_FLEX_ADC = PRE_SEN6

# LKNE_EXT_ADC=3
# LANK_EXT_ADC=4
# RKNE_EXT_ADC=5
# LKNE_FLEX_ADC=7

# LANK_FLEX_ADC=0
# RKNE_FLEX_ADC=1
# RANK_FLEX_ADC=2

# FORCE_ADC =2
# POS_ADC=7
# LKNE_FLEX_ADC=5


#PWM ID, should be the enum PWM_ID value
PCB_VAL_1 = 8
PCB_VAL_2 = 9
PCB_VAL_3 = 10
PCB_VAL_4 = 11
PCB_VAL_5 = 12
PCB_VAL_6 = 13 
PCB_VAL_7 = 14
PCB_VAL_8 = 15
PCB_VAL_9 = 0 
PCB_VAL_10 = 1 
PCB_VAL_11 = 2 
PCB_VAL_12 = 3
PCB_VAL_13 = 4
PCB_VAL_14 = 5
PCB_VAL_15 = 6
PCB_VAL_16 = 7



LKNE_EXUT_PWM=PCB_VAL_1
LTANK_PWM=PCB_VAL_2
RTANK_PWM=PCB_VAL_3
LKNE_EXT_PWM=PCB_VAL_4
LKNE_FLEX_PWM=PCB_VAL_5
RKNE_LANK_PWM=PCB_VAL_6
LANK_EXT_PWM=PCB_VAL_7
RKNE_EXUT_PWM=PCB_VAL_8
LANK_FLEX_PWM=PCB_VAL_9
RANK_FLEX_PWM=PCB_VAL_10
RANK_EXUT_PWM=PCB_VAL_11
LANK_EXUT_PWM=PCB_VAL_12
RANK_EXT_PWM=PCB_VAL_13
LKNE_RANK_PWM=PCB_VAL_14
RKNE_FLEX_PWM=PCB_VAL_15
RKNE_EXT_PWM=PCB_VAL_16

class UdpDataPacket(Structure):
    _fields_=[("pwm_duty",(c_byte)*PWM_VAL_NUM),
              ("enc_data",(c_double)*NUM_ENC),
              ("pre_data1",(c_double)*NUM_PRE),
              ("con_status",(c_bool)*NUM_KNEE_ANK_PAIR),
              ("rec_status",c_bool),
              ("fsm_state",c_int),
              ("is_knee_reverse",c_bool)]
class UdpCmdPacket(Structure):
    _fields_=[("pwm_duty_data",(c_byte)*PWM_VAL_NUM),
              ("lkra_des_pre_data",(c_double)*int(Chamber.MainTank.value)),
              ("rkla_des_pre_data",(c_double)*int(Chamber.MainTank.value)),
            #   ("des_pre_data",(c_double)*(NUM_PRE_CON*NUM_KNEE_ANK_PAIR)),
              ("des_imp_data",(c_double)*(NUM_FORCE_CON*NUM_KNEE_ANK_PAIR)),
              ("des_force_data",(c_double)*(NUM_FORCE_CON*NUM_KNEE_ANK_PAIR)),
              ("epoch_time_data",c_double),
              ("recorder_data",c_bool),
              ("con_on_off_data",(c_bool)*NUM_KNEE_ANK_PAIR),
              ("init_force",(c_double)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("init_impact_imp",(c_double)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("restore_imp",(c_double)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("fsm_left_start",c_bool),
              ("fsm_right_start",c_bool),
              ("fsm_left_swing_left_load_ratio",c_double),
              ("fsm_right_swing_right_load_ratio",c_double),
              ("fsm_left_knee_init_f",c_double),
              ("fsm_right_knee_init_f",c_double),
              ("fsm_left_knee_imp",c_double),
              ("fsm_right_knee_imp",c_double),
              ("fsm_left_ank_idle_pre",c_double),
              ("fsm_right_ank_idle_pre",c_double),
              ("knee_reverse",c_bool),
              ("mpc_train_chamber1",c_int),
              ("mpc_train_chamber2",c_int),
              ("mpc_train_is_lkra",c_bool),


              ("pwm_duty_flag",(c_bool)*PWM_VAL_NUM),
              ("reset_enc_flag",(c_bool)*NUM_ENC),
            #   ("des_pre_flag",(c_bool)*(NUM_KNEE_ANK_PAIR*NUM_PRE_CON)),
              ("lkra_des_pre_flag",(c_bool)*int(Chamber.MainTank.value)),
              ("rkla_des_pre_flag",(c_bool)*int(Chamber.MainTank.value)),
              ("des_imp_flag",(c_bool)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("des_force_flag",(c_bool)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("epoch_time_flag",c_bool),
              ("recorder_flag",c_bool),
              ("con_on_off_flag",(c_bool)*NUM_KNEE_ANK_PAIR),
              ("set_joint_pos_flag",(c_bool)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("impact_absorb_flag",(c_bool)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON)),
              ("set_neutral_flag",c_bool),
              ("fsm_start_flag",c_bool),
              ("fsm_param_change_flag",c_bool),
              ("knee_reverse_flag",c_bool),
              ("mpc_train_gen_flag",c_bool)
              ]
    def __init__(self):
        self.pwm_duty_flag=(ctypes.c_bool*PWM_VAL_NUM)((False)*PWM_VAL_NUM)
        self.reset_enc_flag = (ctypes.c_bool*NUM_ENC)((False)*NUM_ENC)
        self.des_pre_flag = (ctypes.c_bool*(NUM_KNEE_ANK_PAIR*NUM_PRE_CON))((False)*(NUM_KNEE_ANK_PAIR*NUM_PRE_CON))
        self.des_imp_flag = (ctypes.c_bool*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON))((False)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON))
        self.des_force_flag = (ctypes.c_bool*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON))((False)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON))
        self.epoch_time_flag = False
        self.recorder_flag=False
        self.con_on_off_flag = (ctypes.c_bool*NUM_KNEE_ANK_PAIR)((False)*NUM_KNEE_ANK_PAIR)
        self.set_joint_pos_flag = (ctypes.c_bool*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON))((False)*(NUM_KNEE_ANK_PAIR*NUM_FORCE_CON))