#include "FSM.hpp"

FSM& FSM::GetInstance(){
    static FSM instance;
    return instance;
}
void FSM::PushMeas(const double l_hip_s,const double l_knee_s,const double l_ank_s,const double r_hip_s,const double r_knee_s,const double r_ank_s){
    auto &fsm = FSM::GetInstance();
    fsm.cur_joint_vel = fsm.joint_vel_filter.GetFilteredMea(std::array<double,6>{l_hip_s,l_knee_s,l_ank_s,r_hip_s,r_knee_s,r_ank_s});

    fsm.l_hip_s=l_hip_s;
    fsm.l_knee_s = l_knee_s;
    fsm.l_ank_s = l_ank_s;
    fsm.r_hip_s = r_hip_s;
    fsm.r_knee_s = r_knee_s;
    fsm.r_ank_s = r_ank_s;
    
    switch (fsm.cur_state)
    {
    case FSM::State::kLeftLoadRightPush:
        fsm.LeftLoadRightPush();
        break;
    case FSM::State::kLeftStandRightSwing:
        fsm.LeftStandRightSwing();
        break;
    case FSM::State::kLeftStandRightPrep:
        fsm.LeftStandRightPrep();
        break;
    case FSM::State::kRightLoadLeftPush:
        fsm.RightLoadLeftPush();
        break;
    case FSM::State::kRightStandLeftSwing:
        fsm.RightStandLeftSwing();
        break;
    case FSM::State::kRightStandLeftPrep:
        fsm.RightStandLeftPrep();
        break;
    default:
        break;
    }


    
    fsm.fsm_rec.PushData(std::array<int,1>{(int)fsm.cur_state});
    

}
FSM::FSM(/* args */)
:fsm_rec("FSM","Time,state"),
 joint_vel_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b)
{
}

FSM::~FSM()
{
}

FSM::State FSM::GetFSM_State(){
    auto &fsm = FSM::GetInstance();

    return fsm.cur_state;
}

void FSM::LeftLoadRightPush(){
    
}
void FSM::LeftStandRightSwing(){
    
}
void FSM::LeftStandRightPrep(){
    
}
void FSM::RightLoadLeftPush(){
    
}
void FSM::RightStandLeftSwing(){
    
}
void FSM::RightStandLeftPrep(){
    
}

void FSM::TurnOffFSM(){
    FSM::GetInstance().cur_state=State::kNone;
}