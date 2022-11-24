#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP

#include "SensorHub.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "Recorder.hpp"
#include "JointCon.hpp"
#include "MPC.hpp"

enum class PWM_ID //sync with the real connection on the PCB
{

    kLKneExut=PCB_VAL_1,
    kLTank=PCB_VAL_2,
    kRTank=PCB_VAL_3,
    kLKneExt=PCB_VAL_4,
    kLAnkFlex=PCB_VAL_5,
    kLKneAnk=PCB_VAL_6,
    kLAnkExt=PCB_VAL_7,
    kRKneExut=PCB_VAL_8,
    kLKneFlex=PCB_VAL_9,
    kRAnkFlex=PCB_VAL_10,
    kRAnkExt=PCB_VAL_11,
    kLAnkExut=PCB_VAL_12,
    kRKneAnk=PCB_VAL_13,
    kRKneExt=PCB_VAL_14,
    kRKneFlex=PCB_VAL_15,
    kRAnkExut=PCB_VAL_16,
    
};
#define PWM_HEADER "TIME,LKNE_EXUT_PWM,LTANK_PWM,RTANK_PWM,LKNE_EXT_PWM,LANK_FLEX_PWM,LKNE_ANK_PWM,LANK_EXT_PWM,RKNE_EXUT_PWM,LKNE_FLEX_PWM,RANK_FLEX_PWM,RANK_EXT_PWM,LANK_EXUT_PWM,RKNE_ANK_PWM,RKNE_EXT_PWM,RKNE_FLEX_PWM,RANK_EXUT_PWM"
                 
class Valves_hub
{
public:
    enum class Joint //use for force control, impedance control
    {
        kLKne,
        kLAnk,
        kRKne,
        kRAnk,
        kTotal
    };
    enum class Chamber{ //use for pressure control
        kLKneExt,
        kLKneFlex,
        kLAnkExt,
        kLTank,
        kRKneExt,
        kRKneFlex,
        kRAnkExt,
        kRTank,
        kAtoms, //exhaust 
        kTotal

    };

private:
    // MPC Pressure control
    // MPC LTankCon, LKneCon;

    JointCon left_knee_con, right_knee_con, left_ankle_con, right_ankle_con;
    std::array<double,(unsigned)Chamber::kTotal> desired_pre{0};
    std::array<double,(unsigned)Joint::kTotal> desired_imp{0};
    std::array<double,(unsigned)Joint::kTotal> desired_force{0};

    //impact absorb
    std::array<double,(unsigned)Joint::kTotal> init_force{0};
    std::array<double,(unsigned)Joint::kTotal> init_imp{0};
   
    

public:
    

    static Valves_hub &GetInstance();
    // static Valves_hub& GetInstance(std::array<double,SensorHub::NUMENC>&,std::array<double,SensorHub::NUMPRE>&); // ideally this initializer should be called first

    Valves_hub(const Valves_hub &) = delete; // no copy

    static void UpdateValve();
    ~Valves_hub();

    static void SetDuty(uint8_t duty, PWM_ID id);
    static void SetDuty(const std::array<uint8_t, PWM_VAL_NUM> duty);

    // TCP_server read valve condition
    const static std::array<uint8_t, PWM_VAL_NUM> &GetDuty();

    // Basic control function
    static void ReleasePre();

    //Control
    static std::array<bool,(unsigned)Joint::kTotal> GetControlCond();
    static void EnableCon(Joint joint,JointCon::ControlMode mode);
    //Pressure control
    
    
    static void SetDesiredPre(Chamber chamber, double des_pre);
    const static std::array<bool, (unsigned)Joint::kTotal> &GetJointCond();

    // Impdence control
    static void SetDesiredImp(Valves_hub::Joint imp, double imp_val,double init_force);
    static void SetCylnMaxPos(Joint joint);

    //Force
    static void SetDesiredForce(Joint joint, double des_force);

    static void SetJointPos(Joint joint);

    //Impact absorb
    static void SetImpactAbsorb(Valves_hub::Joint joint,double init_force, double init_imp);

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