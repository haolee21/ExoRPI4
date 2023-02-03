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
    kLKneAnk=PCB_VAL_6,  //13
    kLAnkExt=PCB_VAL_7,  //14
    kRKneExut=PCB_VAL_8, //15
    kLAnkFlex=PCB_VAL_9,  //0
    kRAnkFlex=PCB_VAL_10, //1
    kRAnkExut=PCB_VAL_11, //2
    kLAnkExut=PCB_VAL_12, //3
    kRAnkExt=PCB_VAL_13,  //4
    kRKneAnk=PCB_VAL_14,  //5
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
    std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*JointCon::kNumOfChambers> desired_pre{0};
    std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*JointCon::kNumOfChambers> desired_imp{0};
    std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*2> desired_force{0}; //subtanks has no force control
    std::array<double,(unsigned)Valves_hub::KneeAnkPair::kTotal*2> init_force{0};
    
   
    

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

    // Basic control function
    static void ReleasePre();

    //Control
    // static std::array<bool,(unsigned)Joint::kTotal> GetControlCond();
    static void ResetCon(KneeAnkPair joint);
    static void EnableCon(double des_pre,Valves_hub::KneeAnkPair knee_ank_pair,JointCon::PreCon pre_con);
    static void EnableCon(double des_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon force_con_type, JointCon::ForceRedType force_red_type);
    static void EnableCon(double des_imp, double init_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon imp_con_type, JointCon::ForceRedType force_red_type);
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

private:
    Valves_hub();

    std::array<uint8_t, PWM_VAL_NUM> PWM_Duty{0};

    bool valChanged_flag;

    Recorder<uint8_t, PWM_VAL_NUM> pwmRecorder;

    TeensyI2C teensyValveCon;

    // Recorder<double, 11> mpc_ltank_rec; // record p_tank, p_set(target), p_val, q_val
    // Recorder<double, 11> mpc_lkne_rec;
};

#endif