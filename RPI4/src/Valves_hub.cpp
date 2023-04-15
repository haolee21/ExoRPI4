#include "Valves_hub.hpp"
#include "MPC_param.hpp"
#include "FSM.hpp"
Valves_hub::Valves_hub()
    : lkra_con(ExoConfig::GetConfig().left_subtank_knee, ExoConfig::GetConfig().left_subtank_right_ank, ExoConfig::GetConfig().left_knee_right_ank,
               ExoConfig::GetConfig().left_tank_subtank, ExoConfig::GetConfig().left_knee_phy, ExoConfig::GetConfig().right_ankle_phy, "lkra"),
      rkla_con(ExoConfig::GetConfig().right_subtank_knee, ExoConfig::GetConfig().right_subtank_left_ank, ExoConfig::GetConfig().right_knee_left_ank,
               ExoConfig::GetConfig().right_tank_subtank, ExoConfig::GetConfig().right_knee_phy, ExoConfig::GetConfig().left_ankle_phy, "rkla"),
      pwmRecorder("PWM", PWM_HEADER), 
      teensyValveCon(1)
{
    std::cout<<"Valve_hub construct\n";
    // Do not set any valve condition here, it will crash
    // I believe the reason is because TeensyI2C is not created yet
    // I guess the behavior of initialization list is different from I thought
    //  this->mpc_enable.fill(false);
}

Valves_hub::~Valves_hub()
{

    // right now I will just turn off all valves
    std::cout << "Turn off all valves\n";
    Valves_hub::SetDuty(std::array<uint8_t, PWM_VAL_NUM>{0});
    Valves_hub::UpdateValve(); // SetSW or SetDuty only change the flags in Valves_hub
                               // It is UpdateValve that sends them to Teensy
                               // However, when destructor is called, the original callback (RT Timer) has already stopped
                               // Thus, we will have to call it ourself.
}
Valves_hub &Valves_hub::GetInstance()
{
    static Valves_hub instance;
    return instance;
}

void Valves_hub::UpdateValve()
{

Valves_hub &hub = Valves_hub::GetInstance();


    const std::array<double, SensorHub::NUMPRE> &pre_data = SensorHub::GetPreData(); // use ref to avoid copy
    const std::array<double, SensorHub::NUMENC> &enc_data = SensorHub::GetEncData();
   
    hub.lkra_con.PushMeas(pre_data[(unsigned)SensorHub::AdcName::LKneExt],pre_data[(unsigned)SensorHub::AdcName::LKneFLex],pre_data[(unsigned)SensorHub::AdcName::RAnkExt],pre_data[(unsigned)SensorHub::AdcName::RAnkFlex],pre_data[(unsigned)SensorHub::AdcName::LTank],pre_data[(unsigned)SensorHub::AdcName::Tank],
    enc_data[SensorHub::EncName::LKneS],enc_data[SensorHub::EncName::RAnkS],hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk],hub.PWM_Duty[(unsigned)PWM_ID::kLTank]);
    
    hub.rkla_con.PushMeas(pre_data[(unsigned)SensorHub::AdcName::RKneExt],pre_data[(unsigned)SensorHub::AdcName::RKneFlex],pre_data[(unsigned)SensorHub::AdcName::LAnkExt],pre_data[(unsigned)SensorHub::AdcName::LAnkFlex],pre_data[(unsigned)SensorHub::AdcName::RTank],pre_data[(unsigned)SensorHub::AdcName::Tank],
    enc_data[SensorHub::EncName::RKneS],enc_data[SensorHub::EncName::LAnkS],hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt],hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk],hub.PWM_Duty[(unsigned)PWM_ID::kRTank]);

    FSM::Update();
    FSM::State fsm_cur_state = FSM::GetFSM_State();
    if(fsm_cur_state==FSM::State::kLeftSwingRightStand){
        // hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut]=0;
        hub.lkra_con.SetPreControl(FSM::GetRAnkPreParams(),JointCon::Chamber::kSubTank,JointCon::Chamber::kMainTank);



        hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut]=0;
        // hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut]=0;
        
        //in this phase, left ankle is in the air, thus any driving force there is unnecessary
        if(pre_data[(unsigned)SensorHub::AdcName::RKneExt]<pre_data[(unsigned)SensorHub::AdcName::LAnkExt])
            hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=100;
        else
            hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=0;

    }
    else if(fsm_cur_state==FSM::State::kLeftPrepRightStand){
        hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut]=0;

        // if(pre_data[(unsigned)SensorHub::AdcName::RAnkExt]<right_ankle_push_pre)
        hub.lkra_con.SetPreControl(FSM::GetRAnkPreParams(),JointCon::Chamber::kAnkPla,JointCon::Chamber::kSubTank);
        

        hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut]=100;
        
    }

    else if(fsm_cur_state==FSM::State::kLeftLoadRightPush){
        double l_kne_imp;
        double l_kne_initF;
        double l_kne_neu_pos;
        FSM::GetLKneImpParams(l_kne_imp,l_kne_neu_pos,l_kne_initF);
        hub.lkra_con.SetImpControl(JointCon::ForceCon::kKneExt,l_kne_imp,l_kne_initF,l_kne_neu_pos);
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=0;
        
        hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut]=0;
    }
    else if(fsm_cur_state==FSM::State::kLeftStandRightSwing){
        hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut]=0;

        if(pre_data[(unsigned)SensorHub::AdcName::LKneExt]<pre_data[(unsigned)SensorHub::AdcName::RAnkExt])
            hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=100;
        else
            hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=0;



        // hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut]=0;
        
        hub.rkla_con.SetPreControl(FSM::GetLAnkPreParams(),JointCon::Chamber::kSubTank,JointCon::Chamber::kMainTank);



    }
    else if (fsm_cur_state==FSM::State::kLeftStandRightPrep){
        hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex]=100;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut]=100;

        hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut]=0;

        hub.rkla_con.SetPreControl(FSM::GetLAnkPreParams(),JointCon::Chamber::kAnkPla,JointCon::Chamber::kSubTank);

    }


    else if(fsm_cur_state==FSM::State::kLeftPushRightLoad){
        double r_kne_imp;
        double r_kne_initF;
        double r_kne_neu_pos;
        FSM::GetRKneImpParams(r_kne_imp,r_kne_neu_pos,r_kne_initF);
        hub.rkla_con.SetImpControl(JointCon::ForceCon::kKneExt,r_kne_imp,r_kne_initF,r_kne_neu_pos);
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]=0;

        hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex]=0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut]=0;
       


    }
    else if(fsm_cur_state == FSM::State::kTurnOff){
        std::fill_n(hub.PWM_Duty.begin(),hub.PWM_Duty.size(),0);
    }
    


    // hub.valChanged_flag = hub.valChanged_flag || 
    hub.lkra_con.GetValveDuty(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt],hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLTank],hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]);
    // hub.valChanged_flag = hub.valChanged_flag || 
    hub.rkla_con.GetValveDuty(hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt],hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex],hub.PWM_Duty[(unsigned)PWM_ID::kRTank],hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]);


    // auto lkra_con_mode = hub.lkra_con.GetControlMode();

    // if (lkra_con_mode == JointCon::ConMode::kForceCon)
    // {
    //     auto force_con_mode = hub.lkra_con.GetForceImpConMode();
    //     auto force_red_mode = hub.lkra_con.GetForceImpRedMode();
    //     hub.valChanged_flag = true;
    //     switch (force_con_mode)
    //     {
    //     case JointCon::ForceCon::kKneExt:
    //         hub.lkra_con.GetForceCon(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk], hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLTank], force_con_mode, force_red_mode);
    //         break;
    //     default:
    //         break;
    //     }
    // }

    // if (lkra_con_mode == JointCon::ConMode::kImpCon)
    // {
    //     auto imp_con_mode = hub.lkra_con.GetForceImpConMode();
    //     auto imp_red_mode = hub.lkra_con.GetForceImpRedMode();
    //     hub.lkra_con.GetImpCon(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk], hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLTank], imp_con_mode, imp_red_mode);
    //     hub.valChanged_flag = true;
    // }

    // if (hub.valChanged_flag)
    // {
        // std::cout<<"duty: ";
        // check all pwm duty are below 100
        for (int i = 0; i < TeensyI2C::CMDLEN; i++)
        {
            if (hub.PWM_Duty[i] > 100)
                hub.PWM_Duty[i] = 100;
            else if (hub.PWM_Duty[i] < 0)
                hub.PWM_Duty[i] = 0;
            // std::cout<<(int)hub.PWM_Duty[i]<<',';
        }
        // std::cout<<std::endl;

        std::array<char, TeensyI2C::CMDLEN> cmd;

        std::memcpy(cmd.begin(), hub.PWM_Duty.begin(), sizeof(uint8_t) * PWM_VAL_NUM);

        hub.teensyValveCon.WriteCmd(cmd);
        // hub.valChanged_flag = false;
        
    // }
    // put measurements in mpc controller, we must do this even the controller are not enabled since it relies on the history of the measurements
    hub.lkra_con.RecData();
    hub.rkla_con.RecData();
    hub.pwmRecorder.PushData(hub.GetDuty());
}

void Valves_hub::SetDuty(u_int8_t duty, PWM_ID id, Valves_hub::KneeAnkPair knee_ank_pair)
{
    // Valves_hub::ResetCon(knee_ank_pair);
    
        
    Valves_hub &hub = Valves_hub::GetInstance();

    if(knee_ank_pair == KneeAnkPair::kLeftKneeRightAnk){
        hub.lkra_con.ResetControl();
    }
    else{
        hub.rkla_con.ResetControl();
    }

    hub.PWM_Duty[(unsigned)id] = duty;
    // hub.valChanged_flag = true;
}
void Valves_hub::SetDuty(const std::array<u_int8_t, PWM_VAL_NUM> duty)
{
    Valves_hub &hub = Valves_hub::GetInstance();
    std::memcpy(hub.PWM_Duty.begin(), duty.begin(), sizeof(u_int8_t) * PWM_VAL_NUM);
    // hub.valChanged_flag = true;
}

const std::array<uint8_t, PWM_VAL_NUM> &Valves_hub::GetDuty()
{
    return std::ref(Valves_hub::GetInstance().PWM_Duty);
}
void Valves_hub::ShutDownKneAnk(KneeAnkPair joint)
{
    auto &hub = Valves_hub::GetInstance();
    switch (joint)
    {
    case Valves_hub::KneeAnkPair::kLeftKneeRightAnk:
        // std::cout<<"shutdown lkra\n";
        hub.lkra_con.ShutDown();
        break;
    case Valves_hub::KneeAnkPair::kRightKneeLeftAnk:
        // std::cout<<"shutdown rkla\n";
        hub.rkla_con.ShutDown();
        break;
    default:
        break;
    }
}

void Valves_hub::EnableCon(double des_pre, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::Chamber controlled, JointCon::Chamber followed)
{
    FSM::TurnOffFSM();
    
    JointCon *knee_ank_con;
    switch (knee_ank_pair)
    {
    case Valves_hub::KneeAnkPair::kLeftKneeRightAnk:
        
        knee_ank_con = &Valves_hub::GetInstance().lkra_con;
        break;
    case Valves_hub::KneeAnkPair::kRightKneeLeftAnk:
        
        knee_ank_con = &Valves_hub::GetInstance().rkla_con;
        break;
    default:
        return;
    }
    // auto &knee_ank_con = (knee_ank_pair == Valves_hub::KneeAnkPair::kLeftKneeRightAnk) ? hub.lkra_con : hub.rkla_con; //This may cause problems if one day I have more than two knee_ank pairs
    
    knee_ank_con->SetPreControl(des_pre,controlled,followed);

}
void Valves_hub::EnableCon(double des_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon force_con_type)
{
    FSM::TurnOffFSM();
    JointCon *knee_ank_con;
    switch (knee_ank_pair)
    {
    case Valves_hub::KneeAnkPair::kLeftKneeRightAnk:
        knee_ank_con = &Valves_hub::GetInstance().lkra_con;
        break;
    case Valves_hub::KneeAnkPair::kRightKneeLeftAnk:
        knee_ank_con = &Valves_hub::GetInstance().rkla_con;
        break;
    default:
        return;
    }
    knee_ank_con->SetControl(JointCon::ConMode::kForceCon,force_con_type,des_force);

}
void Valves_hub::EnableCon(double des_imp, double init_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon imp_con_type){
    FSM::TurnOffFSM();
    JointCon *knee_ank_con;
    switch (knee_ank_pair)
    {
    case Valves_hub::KneeAnkPair::kLeftKneeRightAnk:
        knee_ank_con = &Valves_hub::GetInstance().lkra_con;
        break;
    case Valves_hub::KneeAnkPair::kRightKneeLeftAnk:
        knee_ank_con = &Valves_hub::GetInstance().rkla_con;
        break;
    default:
        return;
    }
    knee_ank_con->SetImpControl(imp_con_type,des_imp,init_force);
}


void Valves_hub::UpdateParams(const ExoConfig::SystemParam &sys_param)
{
    auto &valves_hub = Valves_hub::GetInstance();
}