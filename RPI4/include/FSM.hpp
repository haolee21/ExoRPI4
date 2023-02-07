#ifndef FSM_HPP
#define FSM_HPP
#include "Recorder.hpp"
#include "DigitalFilter.hpp"
#include "FilterParam.hpp"
class FSM
{

public:
    enum class State{
        kLeftLoadRightPush,
        kLeftStandRightSwing,
        kLeftStandRightPrep,
        kRightLoadLeftPush,
        kRightStandLeftSwing,
        kRightStandLeftPrep,
        kNone
    };
    FSM(const FSM&)=delete;
    static FSM& GetInstance();
    ~FSM();
    static void PushMeas(const double l_hip_s,const double l_knee_s,const double l_ank_s,const double r_hip_s,const double r_knee_s,const double r_ank_s);
    static State GetFSM_State();
    static void TurnOffFSM();
private:

    FSM(/* args */);
    State cur_state;
    Recorder<int,1> fsm_rec;
    void LeftLoadRightPush();
    void LeftStandRightSwing();
    void LeftStandRightPrep();
    void RightLoadLeftPush();
    void RightStandLeftSwing();
    void RightStandLeftPrep();

    //filter for velocity
    DigitalFilter<double,FilterParam::Filter20Hz_2::Order,6> joint_vel_filter;
    std::array<double,6> cur_joint_vel;
    double l_hip_s,l_knee_s,l_ank_s,r_hip_s,r_knee_s,r_ank_s;

};

#endif