#include <iostream>
#include <cmath>
#include "FSM.hpp"

FSM &FSM::GetInstance()
{
    static FSM instance;
    return instance;
}
void FSM::Update()
{
    auto &fsm = FSM::GetInstance();

    double left_leg_traj = fsm.joint_pos[SensorHub::LHipS] - fsm.joint_pos[SensorHub::LKneS]+fsm.joint_pos[SensorHub::LAnkS];
    double right_leg_traj = fsm.joint_pos[SensorHub::RHipS] - fsm.joint_pos[SensorHub::RKneS] + fsm.joint_pos[SensorHub::RAnkS];

    double left_leg_vel = fsm.joint_vel[SensorHub::LHipS]-fsm.joint_vel[SensorHub::LKneS]+fsm.joint_vel[SensorHub::LAnkS];
    double right_leg_vel = fsm.joint_vel[SensorHub::RHipS]-fsm.joint_vel[SensorHub::RKneS]+fsm.joint_vel[SensorHub::RAnkS];

    switch (fsm.cur_state)
    {
    
    case FSM::State::kLeftSwingRightStand:
        if(left_leg_vel<0.8 && left_leg_traj-fsm.l_neu>10)
            fsm.cur_state=State::kLeftPrepRightStand;
        break;
    case FSM::State::kLeftPrepRightStand:
        if(left_leg_vel<-0.5)
            fsm.cur_state=State::kLeftLoadRightPush;
        break;
    case FSM::State::kLeftLoadRightPush:
        if(right_leg_vel>0.8 && left_leg_traj-fsm.l_neu<10 && fsm.joint_vel[SensorHub::RAnkS]>0)
            fsm.cur_state = State::kLeftStandRightSwing;
        break;

    case FSM::State::kLeftStandRightSwing:
        if(right_leg_vel<0.5 && right_leg_traj-fsm.r_neu>10)
            fsm.cur_state = State::kLeftStandRightPrep;
        break;
    case FSM::State::kLeftStandRightPrep:
        if(right_leg_vel<-0.5)
            fsm.cur_state=State::kLeftPushRightLoad;
        break;
    case FSM::State::kLeftPushRightLoad:
        if(left_leg_vel>0 && right_leg_traj-fsm.r_neu<10 && fsm.joint_vel[SensorHub::LAnkS]>0)
            fsm.cur_state=State::kLeftSwingRightStand;
        break;
    default:
        break;
    }

    //record the state so we can see if the FSM switch correctly
    fsm.fsm_rec.PushData(std::array<double, 7>{(double)fsm.cur_state,
                                                fsm.l_hip_s_neu, fsm.l_kne_s_neu, fsm.l_ank_s_neu,
                                                fsm.r_hip_s_neu, fsm.r_kne_s_neu, fsm.r_ank_s_neu});

}
FSM::FSM(/* args */)
    : cur_state(FSM::State::kNone), fsm_rec("FSM", "Time,state,LHip_N,LKne_N,LAnk_N,RHip_N,RKne_N,RAnk_N"),
      joint_pos(SensorHub::GetEncData()),
      joint_vel(SensorHub::GetEncVel())
{
}

FSM::~FSM()
{
}

const FSM::State FSM::GetFSM_State()
{
    auto &fsm = FSM::GetInstance();

    return fsm.cur_state;
}

void FSM::TurnOffFSM()
{
    FSM::GetInstance().cur_state = State::kNone;
    std::cout << "fsm turn off\n";
}

void FSM::SetNetualPos()
{
    auto &fsm = FSM::GetInstance();
    fsm.l_hip_s_neu = fsm.joint_pos[SensorHub::EncName::LHipS];
    fsm.l_kne_s_neu = fsm.joint_pos[SensorHub::EncName::LKneS];
    fsm.l_ank_s_neu = fsm.joint_pos[SensorHub::EncName::LAnkS];
    fsm.r_hip_s_neu = fsm.joint_pos[SensorHub::EncName::RHipS];
    fsm.r_kne_s_neu = fsm.joint_pos[SensorHub::EncName::RKneS];
    fsm.r_ank_s_neu = fsm.joint_pos[SensorHub::EncName::RAnkS];
    std::cout << "set neutral pos\n";
    std::cout << fsm.l_hip_s_neu << ", " << fsm.l_kne_s_neu << ", " << fsm.l_ank_s_neu << std::endl;
    std::cout << fsm.r_hip_s_neu << ", " << fsm.r_kne_s_neu << ", " << fsm.r_ank_s_neu << std::endl;

    fsm.l_neu = fsm.l_hip_s_neu-fsm.l_kne_s_neu+fsm.l_ank_s_neu;
    fsm.r_neu = fsm.r_hip_s_neu-fsm.r_kne_s_neu+fsm.r_ank_s_neu;

}
void FSM::GetLKneImpParams(double &imp, double &neutral_pos, double &init_force)
{
    auto &fsm = FSM::GetInstance();
    imp = fsm.l_kne_s_imp;
    neutral_pos = fsm.l_kne_s_neu;
    init_force = fsm.l_kne_s_initF;
}
void FSM::GetRKneImpParams(double &imp, double &neutral_pos, double &init_force)
{
    auto &fsm = FSM::GetInstance();
    imp = fsm.r_kne_s_imp;
    neutral_pos = fsm.r_kne_s_neu;
    init_force = fsm.r_kne_s_initF;
}
void FSM::SetImpParams(const double left_swing_left_load_ratio, const double right_swing_right_load_ratio, const double l_kne_imp, const double r_kne_imp, const double l_kne_initF, const double r_kne_initF,const double l_ank_idle_pre,const double r_ank_idle_pre)
{
    auto &fsm = FSM::GetInstance();

    fsm.swtich_angle_ratio[(unsigned)FSM::State::kLeftLoadRightPush] = left_swing_left_load_ratio;
    fsm.swtich_angle_ratio[(unsigned)FSM::State::kLeftPushRightLoad] = right_swing_right_load_ratio;

    fsm.r_kne_s_imp = r_kne_imp;
    fsm.l_kne_s_imp = l_kne_imp;
    fsm.r_kne_s_initF = r_kne_initF;
    fsm.l_kne_s_initF = l_kne_initF;

    fsm.l_ank_s_idle_p = l_ank_idle_pre * 320 + 8000; // psi to adc value;
    fsm.r_ank_s_idle_p = r_ank_idle_pre * 320 + 8000; // psi to adc value;
}
void FSM::FSM_LeftStart()
{
    FSM::GetInstance().cur_state = FSM::State::kLeftLoadRightPush;
    std::cout << "left start\n";
}
void FSM::FSM_RightStart()
{
    FSM::GetInstance().cur_state = FSM::State::kLeftPushRightLoad;
    std::cout << "right start\n";
}
double FSM::GetLAnkPreParams(){
    return FSM::GetInstance().l_ank_s_idle_p;
}
double FSM::GetRAnkPreParams(){
    return FSM::GetInstance().r_ank_s_idle_p;
}