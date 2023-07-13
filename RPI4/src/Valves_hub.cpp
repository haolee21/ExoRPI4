#include <random>
#include "Valves_hub.hpp"
#include "MPC_param.hpp"
#include "FSM.hpp"
// #include "Recorder.hpp"
Valves_hub::Valves_hub()
    : lkra_con(ExoConfig::GetConfig().left_subtank_knee, ExoConfig::GetConfig().left_subtank_right_ank, ExoConfig::GetConfig().left_knee_right_ank,
               ExoConfig::GetConfig().left_tank_subtank, ExoConfig::GetConfig().left_knee_phy, ExoConfig::GetConfig().right_ankle_phy, "lkra"),
      rkla_con(ExoConfig::GetConfig().right_subtank_knee, ExoConfig::GetConfig().right_subtank_left_ank, ExoConfig::GetConfig().right_knee_left_ank,
               ExoConfig::GetConfig().right_tank_subtank, ExoConfig::GetConfig().right_knee_phy, ExoConfig::GetConfig().left_ankle_phy, "rkla"),
      pwmRecorder("PWM", PWM_HEADER),
      teensyValveCon(1)
{
    std::cout << "Valve_hub construct\n";
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
void Valves_hub::SetKneeDir(bool is_reverse)
{
    Valves_hub::GetInstance().lkra_con.SetKneeReverse(is_reverse);
    Valves_hub::GetInstance().rkla_con.SetKneeReverse(is_reverse);
}
bool Valves_hub::GetKneeDir()
{
    Valves_hub &hub = Valves_hub::GetInstance();
    bool lkra_reverse = hub.lkra_con.GetKneeReverse();
    bool rkla_reverse = hub.rkla_con.GetKneeReverse();
    if (lkra_reverse ^ rkla_reverse)
        perror("lkra rkla have different knee direction\n");
    return lkra_reverse;
}

void Valves_hub::UpdateValve()
{

    Valves_hub &hub = Valves_hub::GetInstance();

    const std::array<double, SensorHub::NUMPRE> &pre_data = SensorHub::GetPreData(); // use ref to avoid copy
    const std::array<double, SensorHub::NUMENC> &enc_data = SensorHub::GetEncData();

    hub.lkra_con.PushMeas(pre_data[(unsigned)SensorHub::AdcName::LKneExt], pre_data[(unsigned)SensorHub::AdcName::LKneFLex], pre_data[(unsigned)SensorHub::AdcName::RAnkExt], pre_data[(unsigned)SensorHub::AdcName::RAnkFlex], pre_data[(unsigned)SensorHub::AdcName::LTank], pre_data[(unsigned)SensorHub::AdcName::Tank],
                          enc_data[SensorHub::EncName::LKneS], enc_data[SensorHub::EncName::RAnkS], hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt], hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk], hub.PWM_Duty[(unsigned)PWM_ID::kLTank]);

    hub.rkla_con.PushMeas(pre_data[(unsigned)SensorHub::AdcName::RKneExt], pre_data[(unsigned)SensorHub::AdcName::RKneFlex], pre_data[(unsigned)SensorHub::AdcName::LAnkExt], pre_data[(unsigned)SensorHub::AdcName::LAnkFlex], pre_data[(unsigned)SensorHub::AdcName::RTank], pre_data[(unsigned)SensorHub::AdcName::Tank],
                          enc_data[SensorHub::EncName::RKneS], enc_data[SensorHub::EncName::LAnkS], hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt], hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk], hub.PWM_Duty[(unsigned)PWM_ID::kRTank]);

    FSM::Update();
    FSM::State fsm_cur_state = FSM::GetFSM_State();
    //if we switch from double support phase to single support, calculate the desired subtank pressure

    if (fsm_cur_state == FSM::State::kDDLeftFrontRightRear || fsm_cur_state == FSM::State::kRightAnkPushOff)
    {

        // leading knee: impedance control
        // rear ankle: push off
        double l_kne_imp;
        double l_kne_initF;
        double l_kne_neu_pos;
        FSM::GetLKneImpParams(l_kne_imp, l_kne_neu_pos, l_kne_initF);
        hub.lkra_con.SetImpControl(JointCon::ForceCon::kKneExt, l_kne_imp, l_kne_initF, l_kne_neu_pos);
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk] = 0;

        // relax the knee if right knee stops extension
        hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk] = 0;
        // if (fsm_cur_state == FSM::State::kRightAnkPushOff)
        // {
            hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex] = 100;
            hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut] = 100;
        // }
        // else
        // {
        //     hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex] = 0;
        //     hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut] = 0;
        // }
        if(hub.fsm_old_state==FSM::State::kLeftToeOff)
            hub.des_l_subtank_pre = 0;

        double cur_des_l_subtank_pre = hub.lkra_con.GetDesSubTankPre();
        if(cur_des_l_subtank_pre>hub.des_l_subtank_pre)
            hub.des_l_subtank_pre=cur_des_l_subtank_pre;


    }
    else if (fsm_cur_state == FSM::State::kDDRightFrontLeftRear || fsm_cur_state == FSM::State::kLeftAnkPushOff)
    {
        double r_kne_imp;
        double r_kne_initF;
        double r_kne_neu_pos;
        FSM::GetRKneImpParams(r_kne_imp, r_kne_neu_pos, r_kne_initF);
        hub.rkla_con.SetImpControl(JointCon::ForceCon::kKneExt, r_kne_imp, r_kne_initF, r_kne_neu_pos);
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk] = 0;

        hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk] = 0;

        // if (fsm_cur_state == FSM::State::kLeftAnkPushOff)
        // {
            hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex] = 100;
            hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut] = 100;
        // }
        // else
        // {
        //     hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex] = 0;
        //     hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut] = 0;
        // }

        if(hub.fsm_old_state==FSM::State::kRightToeOff)
            hub.des_r_subtank_pre = 0;
        double cur_des_r_subtank_pre = hub.rkla_con.GetDesSubTankPre();
        if(cur_des_r_subtank_pre>hub.des_r_subtank_pre)
            hub.des_r_subtank_pre = cur_des_r_subtank_pre;
    }
    else if (fsm_cur_state == FSM::State::kRightToeOff)
    {
        // if(hub.fsm_old_state == FSM::State::kDDLeftFrontRightRear || hub.fsm_old_state==FSM::State::kRightAnkPushOff)
        //     hub.des_l_subtank_pre = hub.lkra_con.GetDesSubTankPre();

        hub.lkra_con.SetPreControl(hub.des_l_subtank_pre,JointCon::Chamber::kSubTank,JointCon::Chamber::kMainTank);
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex] = 100;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut] = 100;



        hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex] = 100;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut] = 100;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut] = 0;
    }
    else if(fsm_cur_state==FSM::State::kLeftToeOff){

        hub.rkla_con.SetPreControl(hub.des_r_subtank_pre,JointCon::Chamber::kSubTank,JointCon::Chamber::kMainTank);


        // hub.rkla_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kRTank] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneExut] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex] = 100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExut] = 100;

        hub.lkra_con.ResetControl();
        hub.PWM_Duty[(unsigned)PWM_ID::kLTank] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex] = 100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneExut] = 100;
        hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex] = 0;
        hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExut] = 0;
    }

    hub.fsm_old_state = fsm_cur_state;


    hub.lkra_con.GetValveDuty(hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kRAnkExt], hub.PWM_Duty[(unsigned)PWM_ID::kRAnkFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLTank], hub.PWM_Duty[(unsigned)PWM_ID::kLKneRAnk]);
    hub.rkla_con.GetValveDuty(hub.PWM_Duty[(unsigned)PWM_ID::kRKneExt], hub.PWM_Duty[(unsigned)PWM_ID::kRKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLAnkExt], hub.PWM_Duty[(unsigned)PWM_ID::kLAnkFlex], hub.PWM_Duty[(unsigned)PWM_ID::kRTank], hub.PWM_Duty[(unsigned)PWM_ID::kRKneLAnk]);


    //Generate mpc training data
    if(hub.generating_mpc_train){
        if(hub.train_gen_count<kTrainLen){
            hub.PWM_Duty[(unsigned)hub.cur_train_pwm]=hub.mpc_train_duty[hub.train_gen_count];
            hub.train_gen_count++;
        }
        else{
            hub.generating_mpc_train=false;
            hub.PWM_Duty[(unsigned)hub.cur_train_pwm]=0;
        }
    }


    
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

    if (knee_ank_pair == KneeAnkPair::kLeftKneeRightAnk)
    {
        hub.lkra_con.ResetControl();
    }
    else
    {
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

    knee_ank_con->SetPreControl(des_pre, controlled, followed);
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
    knee_ank_con->SetForceControl(force_con_type, des_force);
}
void Valves_hub::EnableCon(double des_imp, double init_force, Valves_hub::KneeAnkPair knee_ank_pair, JointCon::ForceCon imp_con_type)
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
    knee_ank_con->SetImpControl(imp_con_type, des_imp, init_force);
}

void Valves_hub::UpdateParams(const ExoConfig::SystemParam &sys_param)
{
    auto &valves_hub = Valves_hub::GetInstance();
}

void Valves_hub::GenMPC_Train(JointCon::Chamber chamber1, JointCon::Chamber chamber2,bool is_train_lkra){
    Valves_hub &hub = Valves_hub::GetInstance();
    if(hub.generating_mpc_train) //return if it is still generating random duty sequence
        return;

    hub.generating_mpc_train=true;
    hub.train_gen_count=0;
    //generate the random duty sequence
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distribution(15, 80);
    for(auto& duty:hub.mpc_train_duty){
        duty = distribution(gen);
    }
    std::cout<<"duty: ";
    for(const auto&duty:hub.mpc_train_duty){
        std::cout<<duty<<',';
    }
    std::cout<<std::endl;
    if(chamber1==JointCon::Chamber::kMainTank && chamber2==JointCon::Chamber::kSubTank && is_train_lkra){
        hub.cur_train_pwm = PWM_ID::kLTank;
    }
    else if(chamber1==JointCon::Chamber::kMainTank && chamber2==JointCon::Chamber::kSubTank && !is_train_lkra){
        hub.cur_train_pwm=PWM_ID::kRTank;
    }
    else if(chamber1==JointCon::Chamber::kSubTank && chamber2==JointCon::Chamber::kKneExt && is_train_lkra){
        hub.cur_train_pwm=PWM_ID::kLKneExt;
    }
    else if(chamber1==JointCon::Chamber::kSubTank && chamber2==JointCon::Chamber::kKneExt && !is_train_lkra){
        hub.cur_train_pwm=PWM_ID::kRKneExt;
    }
    else if(chamber1==JointCon::Chamber::kKneExt && chamber2==JointCon::Chamber::kAnkPla && is_train_lkra){
        hub.cur_train_pwm=PWM_ID::kLKneRAnk;
    }
    else if(chamber1==JointCon::Chamber::kKneExt && chamber2==JointCon::Chamber::kAnkPla && !is_train_lkra){
        hub.cur_train_pwm=PWM_ID::kRKneLAnk;
    }
    else if(chamber1==JointCon::Chamber::kSubTank && chamber2==JointCon::Chamber::kAnkPla && is_train_lkra){
        
        hub.cur_train_pwm=PWM_ID::kLAnkExt;
    }
    else if(chamber1==JointCon::Chamber::kSubTank && chamber2==JointCon::Chamber::kAnkPla && !is_train_lkra){
        hub.cur_train_pwm=PWM_ID::kRAnkExt;
    }

}