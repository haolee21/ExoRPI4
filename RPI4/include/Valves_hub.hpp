#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP

#include "SensorHub.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "SensorHub.hpp"
#include "Recorder.hpp"
#include "MPC.hpp"

#define NUM_OF_MPC 6
#define NUM_OF_IMP 4
enum PWM_ID
{
    LTANKPRE = PCB_VAL_9,
    LKNEPRE = PCB_VAL_10,
    LANKPRE = PCB_VAL_14,
    RKNEPRE = PCB_VAL_12,
    RANKPRE = PCB_VAL_13,
    RTANKPRE = PCB_VAL_14,
    NA1 = PCB_VAL_15,
    NA2 = PCB_VAL_16,
    NA3 = PCB_VAL_1,
    NA4 = PCB_VAL_2,
    NA5 = PCB_VAL_3,
    NA6 = PCB_VAL_4,
    NA7 = PCB_VAL_5,
    NA8 = PCB_VAL_6,
    NA9 = PCB_VAL_7,
    NA10 = PCB_VAL_8,
    FIRST_PWM = LTANKPRE,
    LAST_PWM = RTANKPRE
};
#define PWM_HEADER "TIME,LTANK_PWM,LKNE_PWM,LKNE_BAL,RKNE_PRE,RANK_PRE,R_TANK" // TODO: need 7 pwm I think, also add LANK back after we finish the imp test
class Valves_hub
{

private:
    // MPC Pressure control
    MPC LTankCon, LKneCon;
    std::array<bool, NUM_OF_MPC> mpc_enable;
    enum class MPC_Enable
    {
        kLTank,
        kLKne,
        kLAnk,
        kRTank,
        kRKne,
        kRAnk
    };
    std::array<bool, NUM_OF_IMP> imp_enable; // flags to enable impedance control

    // bool l_tank_enable,r_tank_enable; //when these flags are true, we will calculate the duty of the pwm during update valve conditions
    std::array<double, PWM_VAL_NUM> desired_pre{0};
    std::array<double, NUM_OF_IMP> desired_imp{0};

public:
    enum class Joint
    {
        kLKne,
        kLAnk,
        kRKne,
        kRAnk
    };

    static Valves_hub &GetInstance();
    // static Valves_hub& GetInstance(std::array<double,SensorHub::NUMENC>&,std::array<double,SensorHub::NUMPRE>&); // ideally this initializer should be called first

    Valves_hub(const Valves_hub &) = delete; // no copy

    static void UpdateValve();
    ~Valves_hub();

    static void SetDuty(uint8_t duty, PWM_ID id);
    static void SetDuty(const std::array<uint8_t, PWM_VAL_NUM> duty);

    // TCP_server read valve condition
    const static std::array<uint8_t, PWM_VAL_NUM> &GetDuty();

    // MPC control
    static void StartMPC(PWM_ID pwm_valve, bool enable);
    static void SetDesiredPre(PWM_ID pwm_valve, double des_pre);
    const static std::array<bool, NUM_OF_MPC> &GetMpcCond();

    // Impdence control
    static void SetDesiredImp(Valves_hub::Joint imp, double imp_val);
    static void SetCylnMaxPos(Joint joint);
    static void EnableImpCon(Valves_hub::Joint imp, bool flag);

private:
    Valves_hub();

    std::array<uint8_t, PWM_VAL_NUM> PWM_Duty{0};

    bool valChanged_flag;

    Recorder<uint8_t, PWM_VAL_NUM> pwmRecorder;

    TeensyI2C teensyValveCon;

    Recorder<double, 11> mpc_ltank_rec; // record p_tank, p_set(target), p_val, q_val
    Recorder<double, 11> mpc_lkne_rec;
};

#endif