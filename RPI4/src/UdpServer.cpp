#include "UdpServer.hpp"
UdpServer::UdpServer(/* args */)
{
    // rx init
    if ((this->sockfd_rx = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("socket create failed");
        exit(EXIT_FAILURE);
    }
    memset((char *)&this->addr_rx, 0, sizeof this->addr_rx);
    this->addr_rx.sin_family = AF_INET;
    this->addr_rx.sin_port = htons(UDP_CMD_PORT);
    this->addr_rx.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(this->sockfd_rx, (struct sockaddr *)&this->addr_rx, sizeof this->addr_rx) == -1)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    // tx init
    if ((this->sockfd_tx = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
    }
    memset((char *)&this->addr_tx, 0, sizeof this->addr_tx);
    this->addr_tx.sin_family = AF_INET;
    this->addr_tx.sin_port = htons(UDP_DATA_PORT);

    this->sockfd_rx_len = sizeof this->addr_rx;
    this->server_thread.reset(new std::thread(&UdpServer::ServerProcess, this));
}
bool UdpServer::Recv(char *buf, int buf_len)
{
    int recv_len = recvfrom(this->sockfd_rx, buf, buf_len, 0, (struct sockaddr *)&this->addr_rx, (socklen_t *)&this->sockfd_rx_len);
    if (recv_len != buf_len)
    {
        std::cout << "recv msg length is wrong\n";
        return false;
    }
    else
    {

        inet_aton(inet_ntoa(this->addr_rx.sin_addr), &this->addr_tx.sin_addr);
        return true;
    }
}
void UdpServer::Send(char *buf, int buf_len)
{
    if (sendto(this->sockfd_tx, buf, buf_len, 0, (struct sockaddr *)&this->addr_tx, sizeof this->addr_tx) == -1)
    {
        perror("UDP Send Failed");
    }
}
void UdpServer::GetDataPacket(UDP_DataPacket &return_packet)
{
    const std::array<double, SensorHub::NUMENC> &enc_data = SensorHub::GetEncData();
    std::copy(enc_data.begin(), enc_data.end(), return_packet.enc_data.begin());
    const std::array<double, SensorHub::NUMPRE> &pre_data = SensorHub::GetPreData();
    std::copy(pre_data.begin(), pre_data.end(), return_packet.pre_data1.begin());

    const std::array<u_int8_t, PWM_VAL_NUM> &pwm_duty = Valves_hub::GetDuty();
    std::copy(pwm_duty.begin(), pwm_duty.end(), return_packet.pwm_duty.begin());

    // const std::array<bool, (unsigned)Valves_hub::Joint::kTotal> controller_cond = Valves_hub::GetControlCond();
    // std::copy(controller_cond.begin(), controller_cond.end(), return_packet.con_status.begin());

    return_packet.recorder = Timer::GetDataRec_flag();
}

UdpServer::~UdpServer()
{
}

bool UdpServer::CheckCmdSet(bool *array, int array_len)
{
    int sum = 0;
    for (int i = 0; i < array_len; i++)
    {
        sum += (int)array[i];
    }

    if (sum == 0)
        return false;
    return true;
}
void UdpServer::ServerProcess()
{
    auto old_time = std::chrono::high_resolution_clock::now();
    while (true)
    {
        UDP_CmdPacket cmd_packet;
        if (this->Recv((char *)&cmd_packet, sizeof cmd_packet))
        {
            UDP_DataPacket data_packet;
            this->GetDataPacket(data_packet);
            this->Send((char *)&data_packet, sizeof data_packet);

            auto cur_time = std::chrono::high_resolution_clock::now();
            auto elapsed = cur_time - old_time;
            long long dur_time = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            old_time = cur_time;
            // handle the commands
            this->ProcessCmd(cmd_packet);
        }
    }
}

void UdpServer::ProcessCmd(UDP_CmdPacket &cmd_packet)
{
    // we check if there is any commands by summing the bool arrays, >0 means there is some commands
    // reset encoder
    if (this->CheckCmdSet(cmd_packet.reset_enc_flag.begin(), cmd_packet.reset_enc_flag.size()))
    {

        if (cmd_packet.reset_enc_flag[SensorHub::EncName::LHipS])
        {
            SensorHub::ResetEnc(SensorHub::EncName::LHipS);
        }
        if (cmd_packet.reset_enc_flag[SensorHub::EncName::LKneS])
        {
            SensorHub::ResetEnc(SensorHub::EncName::LKneS);
        }
        if (cmd_packet.reset_enc_flag[SensorHub::EncName::LAnkS])
        {
            SensorHub::ResetEnc(SensorHub::EncName::LAnkS);
        }
        if (cmd_packet.reset_enc_flag[SensorHub::EncName::RHipS])
        {
            SensorHub::ResetEnc(SensorHub::EncName::RHipS);
        }
        if (cmd_packet.reset_enc_flag[SensorHub::EncName::RKneS])
        {
            SensorHub::ResetEnc(SensorHub::EncName::RKneS);
        }
        if (cmd_packet.reset_enc_flag[SensorHub::EncName::RAnkS])
        {
            SensorHub::ResetEnc(SensorHub::EncName::RAnkS);
        }
    }
    if (this->CheckCmdSet(cmd_packet.pwm_duty_flag.begin(), cmd_packet.pwm_duty_flag.size()))
    {

        // we set the JointCon::ControlMode to kNone to turn off other control process
        //  force control, impedance control, pressure control are turned off by setting duty to 0
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLTank])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLTank], PWM_ID::kLTank, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLKneExt])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLKneExt], PWM_ID::kLKneExt, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLKneFlex])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLKneFlex], PWM_ID::kLKneFlex, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }

        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLKneAnk])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLKneAnk], PWM_ID::kLKneAnk, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLKneExut])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLKneExut], PWM_ID::kLKneExut, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLAnkExut])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLAnkExut], PWM_ID::kLAnkExut, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLAnkExt])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLAnkExt], PWM_ID::kLAnkExt, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kLAnkFlex])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kLAnkFlex], PWM_ID::kLAnkFlex, Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }

        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRTank])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRTank], PWM_ID::kRTank, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRKneExt])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRKneExt], PWM_ID::kRKneExt, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRKneFlex])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRKneFlex], PWM_ID::kRKneFlex, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRKneAnk])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRKneAnk], PWM_ID::kRKneAnk, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRKneExut])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRKneExut], PWM_ID::kRKneExut, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRAnkExut])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRAnkExut], PWM_ID::kRAnkExut, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRAnkExt])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRAnkExt], PWM_ID::kRAnkExt, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
        if (cmd_packet.pwm_duty_flag[(unsigned)PWM_ID::kRAnkFlex])
        {
            Valves_hub::SetDuty(cmd_packet.pwm_duty[(unsigned)PWM_ID::kRAnkFlex], PWM_ID::kRAnkFlex, Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }
    }
    if (this->CheckCmdSet(cmd_packet.des_pre_flag.begin(), cmd_packet.des_pre_flag.size()))
    {

        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kSubTank])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kSubTank] * 320 + 8000, // psi to adc value
                                  Valves_hub::KneeAnkPair::kLeftKneeRightAnk, JointCon::PreCon::kSubTank);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kKneExt])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kKneExt] * 320 + 8000,
                                  Valves_hub::KneeAnkPair::kLeftKneeRightAnk, JointCon::PreCon::kKneExt);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kKneFlex])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kKneFlex] * 320 + 8000,
                                  Valves_hub::KneeAnkPair::kLeftKneeRightAnk, JointCon::PreCon::kKneFlex);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kAnkPlant])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk * 2 + (unsigned)JointCon::PreCon::kAnkPlant] * 320 + 8000,
                                  Valves_hub::KneeAnkPair::kLeftKneeRightAnk, JointCon::PreCon::kAnkPlant);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk * 2 + (unsigned)JointCon::PreCon::kSubTank])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk * 2 + (unsigned)JointCon::PreCon::kSubTank] * 320 + 8000,
                                  Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::PreCon::kSubTank);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk * 2 + (unsigned)JointCon::PreCon::kKneExt])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk * 2 + (unsigned)JointCon::PreCon::kKneExt] * 320 + 8000,
                                  Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::PreCon::kKneExt);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::PreCon::kKneFlex])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::PreCon::kKneFlex]*320+8000,
                                 Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::PreCon::kKneFlex);
        }
        if (cmd_packet.des_pre_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::PreCon::kAnkPlant])
        {
            Valves_hub::EnableCon(cmd_packet.des_pre[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::PreCon::kAnkPlant]*320+8000,
                                 Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::PreCon::kAnkPlant);
        }
    }

    // force control
    if (this->CheckCmdSet(cmd_packet.des_force_flag.begin(), cmd_packet.des_force_flag.size()))
    {
        if (cmd_packet.des_force_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kKneExt])
        {
            Valves_hub::EnableCon(cmd_packet.des_force[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kKneExt],
                                    Valves_hub::KneeAnkPair::kLeftKneeRightAnk, JointCon::ForceCon::kKneExt,JointCon::ForceRedType::kRec);
        }
        if (cmd_packet.des_force_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant])
        {
            Valves_hub::EnableCon(cmd_packet.des_force[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant],
             Valves_hub::KneeAnkPair::kLeftKneeRightAnk, JointCon::ForceCon::kAnkPlant,JointCon::ForceRedType::kBalance);
        }
        if (cmd_packet.des_force_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kKneExt])
        {
            Valves_hub::EnableCon(cmd_packet.des_force[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kKneExt],
              Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::ForceCon::kKneExt,JointCon::ForceRedType::kRec);
        }
        if (cmd_packet.des_force_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant])
        {
            Valves_hub::EnableCon(cmd_packet.des_force[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant],
             Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::ForceCon::kAnkPlant,JointCon::ForceRedType::kBalance);
        }
    }
    // impedance control
    if (this->CheckCmdSet(cmd_packet.des_imp_flag.begin(), cmd_packet.des_imp_flag.size()))
    {
        if (cmd_packet.des_imp_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kKneExt])
        {
            Valves_hub::EnableCon(cmd_packet.des_imp[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kKneExt],
                                  cmd_packet.init_force[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kKneExt],
                                  Valves_hub::KneeAnkPair::kLeftKneeRightAnk,JointCon::ForceCon::kKneExt,JointCon::ForceRedType::kRec);
        }
        else if (cmd_packet.des_imp_flag[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant])
        {
            Valves_hub::EnableCon(cmd_packet.des_imp[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant],
                                  cmd_packet.init_force[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant],
                                  Valves_hub::KneeAnkPair::kLeftKneeRightAnk,JointCon::ForceCon::kAnkPlant,JointCon::ForceRedType::kBalance);
        }
        else if (cmd_packet.des_imp_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kKneExt])
        {
            Valves_hub::EnableCon(cmd_packet.des_imp[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kKneExt],
                                  Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::ForceCon::kKneExt,JointCon::ForceRedType::kRec);
        }
        else if (cmd_packet.des_imp_flag[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant])
        {
            Valves_hub::EnableCon(cmd_packet.des_imp[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk*2+(unsigned)JointCon::ForceCon::kAnkPlant],
                                 Valves_hub::KneeAnkPair::kRightKneeLeftAnk, JointCon::ForceCon::kAnkPlant,JointCon::ForceRedType::kBalance);
        }
    }
    // impact absorb control
    if (this->CheckCmdSet(cmd_packet.impact_absorb_flag.begin(), cmd_packet.impact_absorb_flag.size()))
    {
        // if (cmd_packet.impact_absorb_flag[(unsigned)Valves_hub::Joint::kLKne])
        // {

        //     Valves_hub::SetImpactAbsorb(Valves_hub::Joint::kLKne, cmd_packet.init_force[(unsigned)Valves_hub::Joint::kLKne], cmd_packet.init_impact_imp[(unsigned)Valves_hub::Joint::kLKne]);
        //     Valves_hub::EnableCon(Valves_hub::Joint::kLKne, JointCon::ControlMode::kImpactCon);
        // }
    }

    // update robot's time
    if (cmd_packet.epoch_time_flag)
    {
        // std::cout << "Set DateTime\n";
        // std::cout<< cmd_packet.epoch_time<<std::endl;
        timeval time;
        int input_usec = (cmd_packet.epoch_time - floor(cmd_packet.epoch_time)) * 1000000;

        // std::cout<<input_usec<<std::endl;
        time.tv_sec = (int)cmd_packet.epoch_time;
        time.tv_usec = input_usec;
        settimeofday(&time, NULL);
    }

    // enable recorder, disable recorder
    if (cmd_packet.recorder_flag)
    {
        if (cmd_packet.recorder)
        {
            // enable recorder
            Timer::StartRec();
            std::cout << "start recording\n";
        }
        else
        {
            // disable recorder
            Timer::EndRec();
            std::cout << "stop recording\n";
        }
    }
    // enable joint controller, disable joint controller
    if (this->CheckCmdSet(cmd_packet.con_on_off_flag.begin(), cmd_packet.con_on_off_flag.size()))
    {

        if (!cmd_packet.con_on_off[(unsigned)Valves_hub::KneeAnkPair::kLeftKneeRightAnk])
        {
            Valves_hub::ResetCon(Valves_hub::KneeAnkPair::kLeftKneeRightAnk);
        }

        else if (!cmd_packet.con_on_off[(unsigned)Valves_hub::KneeAnkPair::kRightKneeLeftAnk])
        {
            Valves_hub::ResetCon(Valves_hub::KneeAnkPair::kRightKneeLeftAnk);
        }

    }
    // reset the max pos of each cylinder
    if (this->CheckCmdSet(cmd_packet.set_joint_pos.begin(), cmd_packet.set_joint_pos.size()))
    {
        // if (cmd_packet.set_joint_pos[(unsigned)Valves_hub::Joint::kLKne])
        // {
        //     Valves_hub::SetJointPos(Valves_hub::Joint::kLKne);
        // }
        // if (cmd_packet.set_joint_pos[(unsigned)Valves_hub::Joint::kLAnk])
        // {
        //     Valves_hub::SetJointPos(Valves_hub::Joint::kLAnk);
        // }
        // if (cmd_packet.set_joint_pos[(unsigned)Valves_hub::Joint::kRKne])
        // {
        //     Valves_hub::SetJointPos(Valves_hub::Joint::kRKne);
        // }
        // if (cmd_packet.set_joint_pos[(unsigned)Valves_hub::Joint::kRAnk])
        // {
        //     Valves_hub::SetJointPos(Valves_hub::Joint::kRAnk);
        // }
    }
}