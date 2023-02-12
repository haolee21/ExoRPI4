#include<iostream>
#include "FSM.hpp"
#include "cmath"
FSM& FSM::GetInstance(){
    static FSM instance;
    return instance;
}
void FSM::Update(){
    auto &fsm = FSM::GetInstance();
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
    double hip_diff_vel = fsm.joint_vel[SensorHub::EncName::LHipS]-fsm.joint_vel[SensorHub::EncName::RHipS];
    double hip_diff_pos = fsm.joint_pos[SensorHub::EncName::LHipS]-fsm.joint_pos[SensorHub::EncName::RHipS];
    if((fsm.pre_hip_diff_vel*hip_diff_vel<0)&fsm.leg_switch){
        double lhip_rhip = fsm.joint_pos[SensorHub::EncName::LHipS]-fsm.joint_pos[SensorHub::EncName::RHipS];
        if(lhip_rhip>fsm.kHipDiff){
            fsm.cur_max_lhip_rhip = lhip_rhip;
            fsm.leg_switch=false;
        }
        else if(lhip_rhip<-fsm.kHipDiff){
            fsm.cur_min_lhip_rhip = lhip_rhip;
            fsm.leg_switch=false;
        }
    }
    if(fsm.pre_hip_diff_pos*hip_diff_pos<0)
        fsm.leg_switch=true;


    fsm.cur_stride = fsm.cur_max_lhip_rhip - fsm.cur_min_lhip_rhip;

    fsm.pre_hip_diff_vel = hip_diff_vel;
    fsm.pre_hip_diff_pos = hip_diff_pos;
    
    fsm.fsm_rec.PushData(std::array<double,8>{(double)fsm.cur_state,
                                              fsm.joint_pos[0]-fsm.l_hip_s_neu,fsm.joint_pos[1]-fsm.l_kne_s_neu,fsm.joint_pos[2]-fsm.l_ank_s_neu,
                                              fsm.joint_pos[3]-fsm.r_hip_s_neu,fsm.joint_pos[4]-fsm.r_kne_s_neu,fsm.joint_pos[5]-fsm.r_ank_s_neu,
                                              fsm.cur_stride});
    

}
FSM::FSM(/* args */)
:cur_state(FSM::State::kNone), fsm_rec("FSM","Time,state,LHip_N,LKne_N,LAnk_N,RHip_N,RKne_N,RAnk_N,Stride"),
 joint_pos(SensorHub::GetEncData()),
 joint_vel(SensorHub::GetEncVel())
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
    
    //switch phase based on l_hip_s - r_hip_s
    //if hip_diff < hip_diff_neutral, switch to swing phase
    if(this->joint_pos[SensorHub::LHipS]-this->joint_pos[SensorHub::RHipS]<this->l_hip_s_neu-this->r_hip_s_neu){
        this->cur_state = FSM::State::kLeftStandRightSwing;
        std::cout<<"Phase 0: LeftStandRightSwing\n";
    }

}
void FSM::LeftStandRightSwing(){

    //switch phase based on right knee, if it is extended and the speed is near zero, switch to Prep phase
    // if(this->joint_pos[SensorHub::RKneS]<this->r_kne_s_neu && std::abs(this->joint_vel[SensorHub::RKneS])<this->kVelTh){
    if(this->joint_vel[SensorHub::RHipS]<this->kVelTh){
        this->cur_state = FSM::State::kLeftStandRightPrep;
        std::cout<<"Phase 1: LeftStandRIghtPrep\n";
    }

    
}
void FSM::LeftStandRightPrep(){

    //switch phase based on right ankle, if the ankle dorsiflex more than the neutral point, it switches
    if(this->joint_vel[SensorHub::RKneS]>this->kVelTh){
        this->cur_state=FSM::State::kRightLoadLeftPush;
        std::cout<<"Phase 2: RightLoadLeftPush\n";
    }

    
}
void FSM::RightLoadLeftPush(){

                                                                                                
    //switch phase based on l_hip_s - r_hip_s
    //if hip_diff > hip_diff_neutral, switch to swing phase
    if(this->joint_pos[SensorHub::LHipS]-this->joint_pos[SensorHub::RHipS]>this->l_hip_s_neu-this->r_hip_s_neu){
        this->cur_state = FSM::State::kRightStandLeftSwing;
        std::cout<<"Phase 3: RightStandLeftSwing\n";
    }
    
}
void FSM::RightStandLeftSwing(){

    //switch phase based on left knee, if it is extended and the speed is near zero, switch to Prep phase
    if(this->joint_vel[SensorHub::LHipS]<this->kVelTh){
        this->cur_state = FSM::State::kRightStandLeftPrep;
        std::cout<<"Phase 4: RightStandLeftPrep\n";
    }
    
}
void FSM::RightStandLeftPrep(){
    //switch phase based on right ankle, if the ankle dorsiflex more than the neutral point, it switches
    if(this->joint_vel[SensorHub::LKneS]>this->kVelTh){
        this->cur_state = FSM::State::kLeftLoadRightPush;
        std::cout<<"Phase 5: LeftLoadRightPush\n";
    }
    
}

void FSM::TurnOffFSM(){
    FSM::GetInstance().cur_state=State::kNone;
    std::cout<<"fsm turn off\n";
}

void FSM::SetNetualPos(){
    auto &fsm = FSM::GetInstance();
    fsm.l_hip_s_neu = fsm.joint_pos[SensorHub::EncName::LHipS];
    fsm.l_kne_s_neu = fsm.joint_pos[SensorHub::EncName::LKneS];
    fsm.l_ank_s_neu = fsm.joint_pos[SensorHub::EncName::LAnkS];
    fsm.r_hip_s_neu = fsm.joint_pos[SensorHub::EncName::RHipS];
    fsm.r_kne_s_neu = fsm.joint_pos[SensorHub::EncName::RKneS];
    fsm.r_ank_s_neu = fsm.joint_pos[SensorHub::EncName::RAnkS];
    std::cout<<"set neutral pos\n";
    std::cout<<fsm.l_hip_s_neu<<", "<<fsm.l_kne_s_neu<<", "<<fsm.l_ank_s_neu<<std::endl;
    std::cout<<fsm.r_hip_s_neu<<", "<<fsm.r_kne_s_neu<<", "<<fsm.r_ank_s_neu<<std::endl;

}
void FSM::GetLKneImpParams(double &imp,double &neutral_pos,double &init_force){
    auto& fsm = FSM::GetInstance();
    imp = fsm.l_kne_s_imp;
    neutral_pos = fsm.l_kne_s_neu;
    init_force = fsm.l_kne_s_initF;
}
void FSM::GetRKneImpParams(double &imp,double &neutral_pos,double &init_force){
    auto& fsm = FSM::GetInstance();
    imp = fsm.r_kne_s_imp;
    neutral_pos = fsm.r_kne_s_neu;
    init_force = fsm.r_kne_s_initF;
}
void FSM::SetImpParams(const double r_kne_imp,const double l_kne_imp,const double r_kne_initF,const double l_kne_initF){
    auto& fsm = FSM::GetInstance();
    fsm.r_kne_s_imp = r_kne_imp;
    fsm.l_kne_s_imp = l_kne_imp;
    fsm.r_kne_s_initF = r_kne_initF;
    fsm.l_kne_s_initF = l_kne_initF;
}
void FSM::FSM_LeftStart(){
    FSM::GetInstance().cur_state = FSM::State::kLeftLoadRightPush;
    std::cout<<"left start\n";
}
void FSM::FSM_RightStart(){
    FSM::GetInstance().cur_state = FSM::State::kRightLoadLeftPush;
    std::cout<<"right start\n";
}