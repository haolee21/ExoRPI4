#include <iostream>
#include <cmath>
#include "FSM.hpp"
#include "ExoMomentum.hpp"
FSM &FSM::GetInstance()
{
    static FSM instance;
    return instance;
}
void FSM::Update()
{
    auto &fsm = FSM::GetInstance();

    Eigen::Vector<double, 6> q{0, 0, 0, 0, 0, 0};
    Eigen::Vector<double, 6> dq{0, 0, 0, 0, 0, 0};

    // state kLeftToeOff, kDDLeftFrontRightRear, kRightAnkPushOff,   in these 3 states, lhip>rhip
    // state kRightToeOff,kDDRightFrontLeftRear, kLeftAnkPushOff,    in these 3 states, rhip>lhip
    if (fsm.cur_state == FSM::State::kLeftToeOff || fsm.cur_state == FSM::State::kDDLeftFrontRightRear || fsm.cur_state == FSM::State::kRightAnkPushOff)
    {
        // check if we are off for half gait
        if (fsm.joint_pos[SensorHub::LHipS] - fsm.joint_pos[SensorHub::RHipS] < 0)
        {
            fsm.cur_state = State::kRightToeOff;
        }
        else if (fsm.cur_state == State::kLeftToeOff)
        {
            if (std::abs(fsm.joint_diff[SensorHub::LKneS]) < fsm.kVelTh)
                fsm.cur_state = State::kDDLeftFrontRightRear;
        }
        else if (fsm.cur_state == State::kDDLeftFrontRightRear)
        {
            if (std::abs(fsm.joint_diff[SensorHub::RKneS]) < fsm.kVelTh)
                fsm.cur_state = State::kRightAnkPushOff;
        }
        q = {fsm.joint_pos[SensorHub::LAnkS], fsm.joint_pos[SensorHub::LKneS], fsm.joint_pos[SensorHub::LHipS], fsm.joint_pos[SensorHub::RHipS], fsm.joint_pos[SensorHub::RKneS], fsm.joint_pos[SensorHub::RAnkS]};
        dq = {fsm.joint_diff[SensorHub::LAnkS], fsm.joint_diff[SensorHub::LKneS], fsm.joint_diff[SensorHub::LHipS], fsm.joint_diff[SensorHub::RHipS], fsm.joint_diff[SensorHub::RKneS], fsm.joint_diff[SensorHub::RAnkS]};
    }
    else if (fsm.cur_state == FSM::State::kRightToeOff || fsm.cur_state == FSM::State::kDDRightFrontLeftRear || fsm.cur_state == FSM::State::kLeftAnkPushOff)
    {
        if (fsm.joint_pos[SensorHub::LHipS] - fsm.joint_pos[SensorHub::RHipS] > 0)
        {
            fsm.cur_state = State::kLeftToeOff;
        }

        else if (fsm.cur_state == State::kRightToeOff)
        {
            if (std::abs(fsm.joint_diff[SensorHub::RKneS]) < fsm.kVelTh)
                fsm.cur_state = State::kDDRightFrontLeftRear;
        }
        else if (fsm.cur_state == State::kDDRightFrontLeftRear)
        {
            if (std::abs(fsm.joint_diff[SensorHub::LKneS]) < fsm.kVelTh)
                fsm.cur_state = State::kLeftAnkPushOff;
        }

        q = {fsm.joint_pos[SensorHub::RAnkS], fsm.joint_pos[SensorHub::RKneS], fsm.joint_pos[SensorHub::RHipS], fsm.joint_pos[SensorHub::LHipS], fsm.joint_pos[SensorHub::LKneS], fsm.joint_pos[SensorHub::LAnkS]};
        dq = {fsm.joint_diff[SensorHub::RAnkS], fsm.joint_diff[SensorHub::RKneS], fsm.joint_diff[SensorHub::RHipS], fsm.joint_diff[SensorHub::LHipS], fsm.joint_diff[SensorHub::LKneS], fsm.joint_diff[SensorHub::LAnkS]};
    }

    // TODO:this is just for testing the momentum calculation, remove it after you verify the x irection is correct
    //  q = {fsm.joint_pos[SensorHub::LAnkS],fsm.joint_pos[SensorHub::LKneS],fsm.joint_pos[SensorHub::LHipS],fsm.joint_pos[SensorHub::RHipS],fsm.joint_pos[SensorHub::RKneS],fsm.joint_pos[SensorHub::RAnkS]};
    //  dq = {fsm.joint_diff[SensorHub::LAnkS],fsm.joint_diff[SensorHub::LKneS],fsm.joint_diff[SensorHub::LHipS],fsm.joint_diff[SensorHub::RHipS],fsm.joint_diff[SensorHub::RKneS],fsm.joint_diff[SensorHub::RAnkS]};

    fsm.exo_momentum = sym::Exomomentum<double>(q, dq);
    if (fsm.exo_momentum[1] < fsm.min_y_momentum)
        fsm.min_y_momentum = fsm.exo_momentum[1];
    if (fsm.exo_momentum[1] > fsm.max_y_momentum)
        fsm.max_y_momentum = fsm.exo_momentum[1];
    if (fsm.momentum_erase)
    {
        fsm.pre_y_momentum_loss = fsm.max_y_momentum - fsm.min_y_momentum;
        fsm.min_y_momentum = 0;
        fsm.max_y_momentum = 0;
    }
    // record the state so we can see if the FSM switch correctly
    fsm.fsm_rec.PushData(std::array<double, 10>{(double)fsm.cur_state,
                                                fsm.l_hip_s_neu, fsm.l_kne_s_neu, fsm.l_ank_s_neu,
                                                fsm.r_hip_s_neu, fsm.r_kne_s_neu, fsm.r_ank_s_neu,
                                                fsm.exo_momentum[0], fsm.exo_momentum[1], fsm.pre_y_momentum_loss});
}
FSM::FSM(/* args */)
    : cur_state(FSM::State::kNone), fsm_rec("FSM", "Time,state,LHip_N,LKne_N,LAnk_N,RHip_N,RKne_N,RAnk_N,Momentum_x,Momentum_y,Momentum_y_loss"),
      joint_pos(SensorHub::GetEncData()),
      joint_diff(SensorHub::GetEncDiff())
{
    this->pre_y_momentum_loss = 0;
}

FSM::~FSM()
{
}

const FSM::State FSM::GetFSM_State()
{
    auto &fsm = FSM::GetInstance();

    if (fsm.cur_state == FSM::State::kTurnOff)
    {
        fsm.cur_state = FSM::State::kNone; // return kTurnOff will turn off all the valves, and after that fsm should be in kNone state so valvehub can set pwm duty
        std::cout << "return turn off state\n";
        return FSM::State::kTurnOff;
    }

    return fsm.cur_state;
}

void FSM::TurnOffFSM()
{
    FSM::GetInstance().cur_state = State::kTurnOff;
    std::cout << "fsm turn off\n";
}

void FSM::SetNetualPos()
{
    auto &fsm = FSM::GetInstance();
    // make neutral pos symmetric   /
    //  double hip_neu = (fsm.joint_pos[SensorHub::EncName::LHipS]+fsm.joint_pos[SensorHub::RHipS])/2;
    //  fsm.l_hip_s_neu = hip_neu;
    //  fsm.r_hip_s_neu = hip_neu;
    //  double knee_neu = (fsm.joint_pos[SensorHub::EncName::LKneS]+fsm.joint_pos[SensorHub::RKneS])/2;
    //  fsm.l_kne_s_neu = knee_neu;
    //  fsm.r_kne_s_neu = knee_neu;
    //  double ank_neu = (fsm.joint_pos[SensorHub::EncName::LAnkS]+fsm.joint_pos[SensorHub::RAnkS])/2;
    //  fsm.l_ank_s_neu = ank_neu;
    //  fsm.r_ank_s_neu = ank_neu;

    // it turned out average neutral pos is not working
    fsm.l_hip_s_neu = fsm.joint_pos[SensorHub::EncName::LHipS];
    fsm.r_hip_s_neu = fsm.joint_pos[SensorHub::EncName::RHipS];
    fsm.l_kne_s_neu = fsm.joint_pos[SensorHub::EncName::LKneS];
    fsm.r_kne_s_neu = fsm.joint_pos[SensorHub::EncName::RKneS];
    fsm.l_ank_s_neu = fsm.joint_pos[SensorHub::EncName::LAnkS];
    fsm.r_ank_s_neu = fsm.joint_pos[SensorHub::EncName::RAnkS];

    std::cout << "set neutral pos\n";
    std::cout << fsm.l_hip_s_neu << ", " << fsm.l_kne_s_neu << ", " << fsm.l_ank_s_neu << std::endl;
    std::cout << fsm.r_hip_s_neu << ", " << fsm.r_kne_s_neu << ", " << fsm.r_ank_s_neu << std::endl;
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
void FSM::SetImpParams(const double left_swing_left_load_ratio, const double right_swing_right_load_ratio, const double l_kne_imp, const double r_kne_imp, const double l_kne_initF, const double r_kne_initF, const double l_ank_idle_pre, const double r_ank_idle_pre)
{
    auto &fsm = FSM::GetInstance();

    fsm.r_kne_s_imp = r_kne_imp;
    fsm.l_kne_s_imp = l_kne_imp;
    fsm.r_kne_s_initF = r_kne_initF;
    fsm.l_kne_s_initF = l_kne_initF;

    fsm.l_ank_s_idle_p = l_ank_idle_pre * 320 + 8000; // psi to adc value;
    fsm.r_ank_s_idle_p = r_ank_idle_pre * 320 + 8000; // psi to adc value;
}
void FSM::FSM_LeftStart()
{
    FSM::GetInstance().cur_state = FSM::State::kLeftAnkPushOff;
}
void FSM::FSM_RightStart()
{
    FSM::GetInstance().cur_state = FSM::State::kRightAnkPushOff;
}
double FSM::GetLAnkPreParams()
{
    // return adc value
    return FSM::GetInstance().l_ank_s_idle_p;
}
double FSM::GetRAnkPreParams()
{
    // return adc value
    return FSM::GetInstance().r_ank_s_idle_p;
}