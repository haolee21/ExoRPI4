#include "Valves_hub.hpp"
#include "MPC_param.hpp"
Valves_hub::Valves_hub()
    : lkra_con(ExoConfig::GetConfig().left_subtank_knee, ExoConfig::GetConfig().left_subtank_ank, ExoConfig::GetConfig().left_knee_right_ank,
               ExoConfig::GetConfig().left_tank_subtank, ExoConfig::GetConfig().left_knee_phy, ExoConfig::GetConfig().left_ankle_phy, "lkra"),
      rkla_con(ExoConfig::GetConfig().right_subtank_knee, ExoConfig::GetConfig().right_subtank_ank, ExoConfig::GetConfig().right_knee_left_ank,
               ExoConfig::GetConfig().right_tank_subtank, ExoConfig::GetConfig().right_knee_phy, ExoConfig::GetConfig().right_ankle_phy, "rkla"),
      pwmRecorder("PWM", PWM_HEADER), // TODO: use correct valve names, perhaps adding it in shared file with Teensy
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

    // TODO: add air release sequence
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
    // MPC check
    const std::array<double, SensorHub::NUMPRE> &pre_data = SensorHub::GetPreData(); // use ref to avoid copy
    const std::array<double, SensorHub::NUMENC> &enc_data = SensorHub::GetEncData();
    // TODO: fix this recording
    hub.lkra_con.PushMeas(pre_data[(unsigned)SensorHub::AdcName::LKneExt],pre_data[(unsigned)SensorHub::AdcName::LKneFLex],pre_data[(unsigned)SensorHub::AdcName::LAnkExt],pre_data[(unsigned)SensorHub::AdcName::LTank],pre_data[(unsigned)SensorHub::AdcName::Tank],
    enc_data[SensorHub::EncName::LKneS],enc_data[SensorHub::EncName::LAnkS],hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneAnk],hub.PWM_Duty[(unsigned)PWM_ID::kLTank]);
    
    auto lkra_con_mode = hub.lkra_con.GetControlMode();
    if (lkra_con_mode == JointCon::ConMode::kPreCon)
    {
        auto pre_con_mode = hub.lkra_con.GetPreConMode();
        hub.valChanged_flag = true;
        switch (pre_con_mode)
        {
        case JointCon::PreCon::kKneExt:
            hub.lkra_con.GetPreCon(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], pre_con_mode);
            break;
        case JointCon::PreCon::kKneFlex:
            hub.lkra_con.GetPreCon(hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex], pre_con_mode);
            break;
        case JointCon::PreCon::kSubTank:
            // std::cout<<"call pressure control\n";
            hub.lkra_con.GetPreCon(hub.PWM_Duty[(unsigned)PWM_ID::kLTank], pre_con_mode);
            break;
        default:
            break;
        }
    }
    else if (lkra_con_mode == JointCon::ConMode::kForceCon)
    {
        auto force_con_mode = hub.lkra_con.GetForceImpConMode();
        auto force_red_mode = hub.lkra_con.GetForceImpRedMode();
        hub.valChanged_flag = true;
        switch (force_con_mode)
        {
        case JointCon::ForceCon::kKneExt:
            hub.lkra_con.GetForceCon(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kLKneAnk], hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLTank], force_con_mode, force_red_mode);
            break;
        default:
            break;
        }
    }

    else if (lkra_con_mode == JointCon::ConMode::kImpCon)
    {
        auto imp_con_mode = hub.lkra_con.GetForceImpConMode();
        auto imp_red_mode = hub.lkra_con.GetForceImpRedMode();
        hub.lkra_con.GetImpCon(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kLKneAnk], hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLTank], imp_con_mode, imp_red_mode);
        hub.valChanged_flag = true;
    }

    if (hub.valChanged_flag)
    {
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
        hub.valChanged_flag = false;
    }
    // put measurements in mpc controller, we must do this even the controller are not enabled since it relies on the history of the measurements
    hub.lkra_con.RecData();
    hub.rkla_con.RecData();
    hub.pwmRecorder.PushData(hub.GetDuty());
}

void Valves_hub::SetDuty(u_int8_t duty, PWM_ID id, Valves_hub::KneeAnkPair knee_ank_pair)
{
    Valves_hub::ResetCon(knee_ank_pair);
    Valves_hub &hub = Valves_hub::GetInstance();
    hub.PWM_Duty[(unsigned)id] = duty;
    hub.valChanged_flag = true;
}
void Valves_hub::SetDuty(const std::array<u_int8_t, PWM_VAL_NUM> duty)
{
    Valves_hub &hub = Valves_hub::GetInstance();
    std::memcpy(hub.PWM_Duty.begin(), duty.begin(), sizeof(u_int8_t) * PWM_VAL_NUM);
    hub.valChanged_flag = true;
}

const std::array<uint8_t, PWM_VAL_NUM> &Valves_hub::GetDuty()
{
    return std::ref(Valves_hub::GetInstance().PWM_Duty);
}
void Valves_hub::ResetCon(KneeAnkPair joint)
{
    auto &hub = Valves_hub::GetInstance();
    switch (joint)
    {
    case Valves_hub::KneeAnkPair::kLeftKneeRightAnk:
        hub.lkra_con.ResetControl();
        break;
    case Valves_hub::KneeAnkPair::kRightKneeLeftAnk:
        hub.rkla_con.ResetControl();
        break;
    default:
        break;
    }
}

void Valves_hub::EnableCon(double des_pre, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::PreCon pre_con)
{
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

    knee_ank_con->SetControl(JointCon::ConMode::kPreCon, pre_con, des_pre);

}
void Valves_hub::EnableCon(double des_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon force_con_type, JointCon::ForceRedType force_red_type)
{
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
    knee_ank_con->SetControl(JointCon::ConMode::kForceCon,force_con_type,force_red_type,des_force);

}
void Valves_hub::EnableCon(double des_imp, double init_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon imp_con_type, JointCon::ForceRedType force_red_type){
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
    knee_ank_con->SetControl(JointCon::ConMode::kImpCon,imp_con_type,force_red_type,des_imp,init_force);
}
// void Valves_hub::EnableCon(KneeAnkPair knee_ank,JointCon::ConMode mode){
//     Valves_hub& hub = Valves_hub::SGetInstance();

//     if(knee_ank == Valves_hub::KneeAnkPair::kLeftKneeRightAnk){
//         hub.lkra_con.SetControlMode(mode);
//     }
//     else if(knee_ank ==Valves_hub::KneeAnkPair::kRightKneeLeftAnk){
//         hub.rkla_con.SetControlMode(mode);
//     }
// }

// void Valves_hub::SetDesiredPre(Chamber chamber,double des_pre){
//     Valves_hub& hub = Valves_hub::GetInstance();
//     hub.desired_pre[(unsigned)chamber] = des_pre;

// }
// void Valves_hub::SetDesiredImp(Valves_hub::Joint imp,double imp_val,double init_force){ //TODO: finish it
//     Valves_hub& hub = Valves_hub::GetInstance();
//     hub.desired_imp[static_cast<unsigned>(imp)]=imp_val;
//     hub.init_force[(unsigned)imp]=init_force;
// }
// void Valves_hub::SetDesiredForce(Valves_hub::Joint joint, double des_force){
//     Valves_hub& hub = Valves_hub::GetInstance();
//     hub.desired_force[(unsigned)joint]=des_force;
// }

// void Valves_hub::SetCylnMaxPos(Valves_hub::Joint joint){
//     // switch (joint)
//     // {
//     // case Valves_hub::Joint::kLKne:
//     //     Valves_hub::GetInstance().LKneCon.SetCylinderMaxPos();
//     //     break;
//     // //TODO: finish the rest of the joints
//     // default:
//     //     break;
//     // }

// }
// std::array<bool,(unsigned)Valves_hub::Joint::kTotal>Valves_hub::GetControlCond(){
//     Valves_hub& hub = Valves_hub::GetInstance();
//     std::array<bool,(unsigned)Valves_hub::Joint::kTotal> cur_cond{0};
//     if(hub.left_knee_con.GetControlMode()!=JointCon::ConMode::kNone){
//         cur_cond[0]=true;
//     }
//     if(hub.left_ankle_con.GetControlMode()!=JointCon::ConMode::kNone){
//         cur_cond[1]=true;
//     }
//     return cur_cond;
// }

// void Valves_hub::SetJointPos(Valves_hub::Joint joint){
//     auto &valves_hub = Valves_hub::GetInstance();
//     auto &sensor_hub = SensorHub::GetInstance();
//     //TODO: fix this
//     // if(joint == Valves_hub::Joint::kLKne){
//     //     valves_hub.left_knee_con.SetKneeMaxPos(sensor_hub.GetPreData()[(unsigned)SensorHub::AdcName::Pos]);
//     // }
// }

// void Valves_hub::SetImpactAbsorb(Valves_hub::Joint joint, double init_force, double init_imp){
//     auto &valves_hub = Valves_hub::GetInstance();
//     valves_hub.init_force[(unsigned)joint]=init_force;
//     valves_hub.init_imp[(unsigned)joint]=init_imp;
// }

void Valves_hub::UpdateParams(const ExoConfig::SystemParam &sys_param)
{
    auto &valves_hub = Valves_hub::GetInstance();
}