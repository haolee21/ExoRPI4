#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP

#include "SensorHub.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "SensorHub.hpp"
#include "Recorder.hpp"
#include "JointCon.hpp"
#include "MPC.hpp"

enum class PWM_ID //sync with the real connection on the PCB
{
    kLTank = PCB_VAL_9,
    kLKneExt = PCB_VAL_10,
    kLKneFlex = PCB_VAL_11,
    kLRel = PCB_VAL_12,
    kLAnkExt = PCB_VAL_13,
    kLAnkFlex= PCB_VAL_14,
    kRTank = PCB_VAL_15,
    kRKneExt = PCB_VAL_16,
    kRKneFlex = PCB_VAL_1,
    kRAnkExt = PCB_VAL_2,
    kRAnkFlex = PCB_VAL_3,
    NA1 = PCB_VAL_4,
    NA2 = PCB_VAL_5,
    NA3 = PCB_VAL_6,
    NA4 = PCB_VAL_7,
    NA5 = PCB_VAL_8,
    NA6 = PCB_VAL_9,
};
#define PWM_HEADER "TIME,LTANK_PWM,LKNE_EXT_PWM,LKNE_FLEX_PWM,LREL_PWM,LANK_EXT_PWM,LANK_FLEX_PWM,RTANK_PWM,RKNE_EXT_PWM,RKEN_FLEX_PWM,RANK_EXT_PWM,RANK_FLEX_PWM,NA1_PWM,NA2_PWM,NA3_PWN,NA4_PWM,NA5_PWM,NA6_PWM" // TODO: need 7 pwm I think, also add LANK back after we finish the imp test
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
        kLAnkFlex,
        kLTank,
        kRKneExt,
        kRKneFlex,
        kRAnkExt,
        kRAnkFlex,
        kRTank,
        kTotal

    };

private:
    // MPC Pressure control
    // MPC LTankCon, LKneCon;

    JointCon left_knee_con, right_knee_con, left_ankle_con, right_ankle_con;
    std::array<double,(unsigned)Chamber::kTotal> desired_pre{0};
    std::array<double,(unsigned)Joint::kTotal> desired_imp{0};
    std::array<double,(unsigned)Joint::kTotal> desired_force{0};

    

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
    static void SetDesiredImp(Valves_hub::Joint imp, double imp_val);
    static void SetCylnMaxPos(Joint joint);

    //Force
    static void SetDesiredForce(Joint joint, double des_force);

    

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