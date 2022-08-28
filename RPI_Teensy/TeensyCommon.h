#define VALVE_CON_ADDRESS 0x002D
#define PWM_VAL_NUM 16
// #define SW_VAL_NUM 8

#define PWM_HEADER "TIME,LTANK_PWM,LKNE_PWM,LKNE_BAL,RKNE_PRE,RANK_PRE,R_TANK" //TODO: need 7 pwm I think, also add LANK back after we finish the imp test
#define TEENSY_PWM1 8   //VAL 9
#define PCB_VAL_9 0   //this is the index when we send the array to teensy

#define TEENSY_PWM2 9 //VAL 10
#define PCB_VAL_10 1

#define TEENSY_PWM3 23 //VAL 11
#define PCB_VAL_11 2

#define TEENSY_PWM4 22 //VAL 12
#define PCB_VAL_12 3

#define TEENSY_PWM5 21 //VAL 13
#define PCB_VAL_13 4

#define TEENSY_PWM6 14 //VAL 14
#define PCB_VAL_14 5

#define TEENSY_PWM7 15   //VAL 15
#define PCB_VAL_15 6

#define TEENSY_PWM8 16    //VAL 16
#define PCB_VAL_16 7

#define SW_HEADER "TIME,LBAL,LREL,RBAL,RREL,TANK_REL,NA,NA,NA" //may change
#define TEENSY_PWM9 0 //VAL 1
#define PCB_VAL_1 8 

#define TEENSY_PWM10 1 //VAL 2
#define PCB_VAL_2 9

#define TEENSY_PWM11 2 //VAL 3
#define PCB_VAL_3 10

#define TEENSY_PWM12 3 //VAL 4
#define PCB_VAL_4 11

#define TEENSY_PWM13 4 //VAL 5
#define PCB_VAL_5 12 

#define TEENSY_PWM14 5 //VAL 6
#define PCB_VAL_6 13 

#define TEENSY_PWM15 6 //VAL 7
#define PCB_VAL_7 14 

#define TEENSY_PWM16 7 //VAL 8
#define PCB_VAL_8 15 

