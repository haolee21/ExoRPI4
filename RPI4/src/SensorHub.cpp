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
    return std::ref(SensorHub::GetInstance().EncData);
}
const std::array<u_int16_t, SensorHub::NUMPRE> &SensorHub::GetPreData()
{
    return std::ref(SensorHub::GetInstance().PreData);
    // return std::cref(SensorHub::GetInstance().adc0.ReadData());
}

SensorHub::~SensorHub()
{
    //we will have to release all the memory we locked in Start();
    munlockall();
}
SensorHub::SensorHub() //initialize member in list since Encoder has no default constructor
    : LEncRecorder(Recorder<uint16_t,NUMENC/2>("EncodersL","LHipS,LKneS,LAnkS"))
    , REncRecorder(Recorder<uint16_t,NUMENC/2>("EncodersR","RHipS,RKneS,RAnkS"))
    ,PreRecorder(Recorder<uint16_t,NUMPRE>("Pressure","test"))
    , LHipS_Enc(Encoder_L(Encoder_L::HIP1)), LKneS_Enc(Encoder_L(Encoder_L::KNEE)), LAnkS_Enc(Encoder_L(Encoder_L::ANK1)), RHipS_Enc(Encoder_R(Encoder_R::HIP1)), RKneS_Enc(Encoder_R(Encoder_R::KNEE)), RAnkS_Enc(Encoder_R(Encoder_R::ANK1)),adc0(ADC(0))//,adc1(ADC(1)) //, LHipF_Enc(Encoder_L(1)), LAnkF_Enc(Encoder_L(4)), RHipF_Enc(Encoder_R(1)), RAnkF_Enc(Encoder_R(4))
{
    
}

void SensorHub::ResetEncImpl(SensorHub::EncName EncName)
{
    switch (EncName)
    {
    // case SensorHub::EncName::LHipF:
    //     this->LHipF_Enc.SetZero();
    //     break;
    case SensorHub::EncName::LHipS:
        this->LHipS_Enc.SetZero();
        break;
    case SensorHub::EncName::LKneS:
        this->LKneS_Enc.SetZero();
        break;
    case SensorHub::EncName::LAnkS:
        this->LAnkS_Enc.SetZero();
        break;
    // case SensorHub::EncName::LAnkF:
    //     this->LAnkF_Enc.SetZero();
    //     break;
    // case SensorHub::EncName::RHipF:
    //     this->RHipF_Enc.SetZero();
    //     break;
    case SensorHub::EncName::RHipS:
        this->RHipS_Enc.SetZero();
        break;
    case SensorHub::EncName::RKneS:
        this->RKneS_Enc.SetZero();
        break;
    case SensorHub::EncName::RAnkS:
        this->RAnkS_Enc.SetZero();
        break;
    // case SensorHub::EncName::RAnkF:
    //     this->RAnkF_Enc.SetZero();
    }
}

void SensorHub::UpdateLEnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    // senHub.EncData[SensorHub::LHipS] = senHub.LHipS_Enc.ReadPos();   //TODO: add robustness to ReadPos() when encoder is offline
    // senHub.EncData[SensorHub::LKneS] = senHub.LKneS_Enc.ReadPos();   //TODO: read the correct encoder when encoders connected
    // senHub.EncData[SensorHub::LAnkS] = senHub.LAnkS_Enc.ReadPos();
    std::array<uint16_t,NUMENC/2> curMea{senHub.EncData[SensorHub::LHipS],senHub.EncData[SensorHub::LKneS],senHub.EncData[SensorHub::LAnkS]};
    
    senHub.LEncRecorder.PushData(curMea);
    // senHub.EncData[SensorHub::LHipF] = senHub.LAnkS_Enc.ReadPos();
    // senHub.EncData[SensorHub::LAnkF] = senHub.LAnkS_Enc.ReadPos();    
}
void SensorHub::UpdateREnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    // senHub.EncData[SensorHub::RHipS]=senHub.RHipS_Enc.ReadPos();   ////TODO: read the correct encoder when encoders connected
    // senHub.EncData[SensorHub::RHipF]=senHub.RKneS_Enc.ReadPos();
    senHub.EncData[SensorHub::RKneS]=senHub.RKneS_Enc.ReadPos();
    // senHub.EncData[SensorHub::RAnkS]=senHub.RAnkS_Enc.ReadPos();
    // senHub.EncData[SensorHub::RAnkF]=senHub.RKneS_Enc.ReadPos();
    std::array<uint16_t,NUMENC/2> curMea{senHub.EncData[SensorHub::RHipS],senHub.EncData[SensorHub::RKneS],senHub.EncData[SensorHub::RAnkS]};
    senHub.REncRecorder.PushData(curMea);
}
void SensorHub::UpdatePre()
{
    SensorHub & senHub = SensorHub::GetInstance();
    const std::array<u_int16_t,8> &data = senHub.adc0.ReadData();
    senHub.PreData[0]=data[ADC::SEN1];
    senHub.PreData[1]=data[ADC::SEN2];
    senHub.PreData[2]=data[ADC::SEN3];
    senHub.PreData[3]=data[ADC::SEN4];
    senHub.PreData[4]=data[ADC::SEN5];


  
}

