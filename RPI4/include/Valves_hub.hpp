#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP

#include "SensorHub.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "Recorder.hpp"
#include "JointCon.hpp"
#include "MPC.hpp"
#include "ExoConfig.hpp"
enum class PWM_ID //sync with the real connection on the PCB
{
    kLKneExut=PCB_VAL_1,  //8
    kLTank=PCB_VAL_2,     //9
    kRTank=PCB_VAL_3,    //10
    kLKneExt=PCB_VAL_4,  //11
    kLKneFlex=PCB_VAL_5, //12
    kRKneLAnk=PCB_VAL_6,  //13
    kLAnkExt=PCB_VAL_7,  //14
    kRKneExut=PCB_VAL_8, //15
    kLAnkFlex=PCB_VAL_9,  //0
    kRAnkFlex=PCB_VAL_10, //1
    kRAnkExut=PCB_VAL_11, //2
    kLAnkExut=PCB_VAL_12, //3
    kRAnkExt=PCB_VAL_13,  //4
    kLKneRAnk=PCB_VAL_14,  //5
    kRKneFlex=PCB_VAL_15, //6
    kRKneExt=PCB_VAL_16,  //7
    
};
#define PWM_HEADER "TIME,LANK_DOR_PWM,RANK_DOR_PWM,RANK_EXUT_PWM,LANK_EXUT_PWM,RANK_PLA_PWM,LKNE_RANK_PWM,RKNE_FLEX_PWM,RKNE_EXT_PWM,LKNE_EXUT_PWM,LTANK_PWM,RTANK_PWM,LKNE_EXT_PWM,LKNE_FLEX_PWM,RKNE_LANK_PWM,LANK_PLA_PWM,RKNE_EXUT_PWM"

class Valves_hub
{
public:
    enum class KneeAnkPair{
        kLeftKneeRightAnk,
        kRightKneeLeftAnk,
        kTotal
    };
    


private:
    // MPC Pressure control
    // MPC LTankCon, LKneCon;

    JointCon lkra_con, rkla_con; //left-knee-right-ankle, left-knee-right-ankle
    // std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*JointCon::kNumOfChambers> desired_pre{0};
    // std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*JointCon::kNumOfChambers> desired_imp{0};
    // std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*2> desired_force{0}; //subtanks has no force control
    // std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*2> init_force{0};
    
   
    

public:
    

    static Valves_hub &GetInstance();
    // static Valves_hub& GetInstance(std::array<double,SensorHub::NUMENC>&,std::array<double,SensorHub::NUMPRE>&); // ideally this initializer should be called first

    Valves_hub(const Valves_hub &) = delete; // no copy

    static void UpdateValve();
    ~Valves_hub();

    static void SetDuty(u_int8_t duty, PWM_ID id,KneeAnkPair knee_ank_pair);
    static void SetDuty(const std::array<u_int8_t, PWM_VAL_NUM> duty);

    // TCP_server read valve condition
    const static std::array<u_int8_t, PWM_VAL_NUM> &GetDuty();

  

    //Control
    // static std::array<bool,(unsigned)Joint::kTotal> GetControlCond();
    static void ShutDownKneAnk(KneeAnkPair joint);
    static void EnableCon(double des_pre,Valves_hub::KneeAnkPair knee_ank_pair, JointCon::Chamber controlled, JointCon::Chamber followed);
    static void EnableCon(double des_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon force_con_type);
    static void EnableCon(double des_imp, double init_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon imp_con_type);
    //Pressure control
    
    
    // static void SetDesiredPre(Chamber chamber, double des_pre);
    // const static std::array<bool, (unsigned)Joint::kTotal> &GetJointCond();

    // Impdence control
    // static void SetDesiredImp(Valves_hub::Joint imp, double imp_val,double init_force);
    // static void SetCylnMaxPos(Joint joint);

    //Force
    // static void SetDesiredForce(Joint joint, double des_force);

    // static void SetJointPos(Joint joint);

    //Impact absorb
    // static void SetImpactAbsorb(Valves_hub::Joint joint,double init_force, double init_imp);

    // Update MPC parameters
    static void UpdateParams(const ExoConfig::SystemParam &sys_param);
    // Update knee direction
    static void SetKneeDir(bool is_reverse); //backward knee: is_reverse=true
    static bool GetKneeDir();

    static void GenMPC_Train(JointCon::Chamber chamber1, JointCon::Chamber chamber2,bool is_train_lkra);
private:
    Valves_hub();

    std::array<uint8_t, PWM_VAL_NUM> PWM_Duty{0};

    // bool valChanged_flag;

    Recorder<uint8_t, PWM_VAL_NUM> pwmRecorder;

    TeensyI2C teensyValveCon;

    
    //FSM related parameters
    // desired subtank pressure for the next cycle
    double des_l_subtank_pre, des_r_subtank_pre; 
    FSM::State fsm_old_state = FSM::State::kNone;


    //MPC Param Training
    constexpr static int kTrainLen=200+MPC_TIME_HORIZON; //training data is 2 sec with 100 Hz sampling rate
    int train_gen_count=0; //when generating training sets, it will increase from 0 to 200
    bool generating_mpc_train=false;
    
    PWM_ID cur_train_pwm;
    JointCon::Chamber mpc_train_chamber1, mpc_train_chamber2;
    std::array<int,kTrainLen> mpc_train_duty;
    
};

#endif