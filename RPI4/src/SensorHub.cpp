#include "SensorHub.hpp"
#include <functional>


#include "RT.hpp"

#include <future>
#define MEMSIZE (100 * 1024) // 100kB
// static functions
SensorHub &SensorHub::GetInstance()
{
    static SensorHub instance;
    return instance;
}
void SensorHub::ResetEnc(SensorHub::EncName encName)
{
    SensorHub::GetInstance().ResetEncImpl(encName);
}
const std::array<u_int16_t, SensorHub::NUMENC> &SensorHub::GetEncData()
{
    return std::cref(SensorHub::GetInstance().EncData);
}
const std::array<u_int16_t, SensorHub::NUMPRE> &SensorHub::GetPreData()
{
    return std::cref(SensorHub::GetInstance().adc0.ReadData());
}

SensorHub::~SensorHub()
{
    //we will have to release all the memory we locked in Start();
    munlockall();
}
SensorHub::SensorHub() //initialize member in list since Encoder has no default constructor
    : LHipS_Enc(Encoder_L(0)), LHipF_Enc(Encoder_L(1)), LKneS_Enc(Encoder_L(2)), LAnkS_Enc(Encoder_L(3)), LAnkF_Enc(Encoder_L(4)), RHipS_Enc(Encoder_R(0)), RHipF_Enc(Encoder_R(1)), RKneS_Enc(Encoder_R(2)), RAnkS_Enc(Encoder_R(3)), RAnkF_Enc(Encoder_R(4)),adc0(ADC(0)),adc1(ADC(1))
{
    
}

void SensorHub::ResetEncImpl(SensorHub::EncName EncName)
{
    switch (EncName)
    {
    case SensorHub::EncName::LHipF:
        this->LHipF_Enc.SetZero();
        break;
    case SensorHub::EncName::LHipS:
        this->LHipS_Enc.SetZero();
        break;
    case SensorHub::EncName::LKneS:
        this->LKneS_Enc.SetZero();
        break;
    case SensorHub::EncName::LAnkS:
        this->LAnkS_Enc.SetZero();
        break;
    case SensorHub::EncName::LAnkF:
        this->LAnkF_Enc.SetZero();
        break;
    case SensorHub::EncName::RHipF:
        this->RHipF_Enc.SetZero();
        break;
    case SensorHub::EncName::RHipS:
        this->RHipS_Enc.SetZero();
        break;
    case SensorHub::EncName::RKneS:
        this->RKneS_Enc.SetZero();
        break;
    case SensorHub::EncName::RAnkS:
        this->RAnkS_Enc.SetZero();
        break;
    case SensorHub::EncName::RAnkF:
        this->RAnkF_Enc.SetZero();
    }
}

void SensorHub::UpdateLEnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    senHub.EncData[SensorHub::LHipS] = senHub.LAnkF_Enc.ReadPos();   //TODO: add robustness to ReadPos() when encoder is offline
    senHub.EncData[SensorHub::LHipF] = senHub.LAnkF_Enc.ReadPos();   //TODO: read the correct encoder when encoders connected
    senHub.EncData[SensorHub::LKneS] = senHub.LAnkF_Enc.ReadPos();
    senHub.EncData[SensorHub::LAnkS] = senHub.LAnkF_Enc.ReadPos();
    senHub.EncData[SensorHub::LAnkF] = senHub.LAnkF_Enc.ReadPos();    
}
void SensorHub::UpdateREnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    senHub.EncData[SensorHub::RHipS]=senHub.RKneS_Enc.ReadPos();   ////TODO: read the correct encoder when encoders connected
    senHub.EncData[SensorHub::RHipF]=senHub.RKneS_Enc.ReadPos();
    senHub.EncData[SensorHub::RKneS]=senHub.RKneS_Enc.ReadPos();
    senHub.EncData[SensorHub::RAnkS]=senHub.RKneS_Enc.ReadPos();
    senHub.EncData[SensorHub::RAnkF]=senHub.RKneS_Enc.ReadPos();
    
}
// void SensorHub::UpdatePre()
// {
//     SensorHub & senHub = SensorHub::GetInstance();
// }

