#ifndef FSM_HPP
#define FSM_HPP
#include "Recorder.hpp"
#include "DigitalFilter.hpp"
#include "FilterParam.hpp"
#include "SensorHub.hpp"
class FSM
{

public:
    enum class State{
        kLeftSwingRightStand,
        // kLeftPrepRightStand,
        kLeftLoadRightPush,
        kLeftStandRightSwing,
        // kLeftStandRightPrep,
        kLeftPushRightLoad,
        kNone
    };
    
    FSM(const FSM&)=delete;
    static FSM& GetInstance();
    ~FSM();
    static void Update();
    static State GetFSM_State();
    static void FSM_LeftStart();
    static void FSM_RightStart();
    static void TurnOffFSM();
    static void SetNetualPos();

    //FSM control params
    static void GetLKneImpParams(double &imp,double &neutral_pos,double &init_force);
    static void GetRKneImpParams(double &imp,double &neutral_pos,double &init_force);
    static void SetImpParams(const double left_swing_left_load_ratio,const double right_swing_right_load_ratio, const double l_kne_imp,const double r_kne_imp,const double l_kne_initF,const double r_kne_initF);
    
private:
    std::array<double,(unsigned)State::kNone>swtich_angle_ratio={0.0,0.65,0,0.56};
    FSM(/* args */);
    State cur_state;
    Recorder<double,10> fsm_rec;
    void LeftLoadRightPush();
    void LeftStandRightSwing();
    // void LeftStandRightPrep();
    void LeftPushRightLoad();
    void LeftSwingRightStand();
    // void RightStandLeftPrep();

    const std::array<double,SensorHub::NUMENC> &joint_pos;
    const std::array<double,SensorHub::NUMENC> &joint_vel;
    const double kVelTh = 0.01; //if less than  1/100 deg/sec, consider stop
    const double kHipDiff = 10;                                                                            

    double l_hip_s_neu,l_kne_s_neu,l_ank_s_neu,r_hip_s_neu,r_kne_s_neu,r_ank_s_neu;

    double l_kne_s_imp,l_ank_s_imp,r_kne_s_imp,r_ank_s_imp;
    double l_kne_s_initF,r_kne_s_initF;

    double cur_max_lhip_rhip,cur_min_lhip_rhip;
    double hip_diff_vel,hip_diff_pos;
    bool leg_switch=true;
    bool reach_peak = false;
    double cur_stride;

};

#endif