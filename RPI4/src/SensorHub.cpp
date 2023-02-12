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
const std::array<double, SensorHub::NUMENC> &SensorHub::GetEncData()
{
    return std::ref(SensorHub::GetInstance().EncData);
}
const std::array<double,SensorHub::NUMENC>& SensorHub::GetEncVel()
{   
    return std::ref(SensorHub::GetInstance().EncVel);
}
const std::array<double, SensorHub::NUMPRE> &SensorHub::GetPreData()
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
    : LEncRecorder("EncodersL","Time,LHipS,LKneS,LAnkS,LHipS_v,LKneS_v,LAnkS_v")
    , REncRecorder("EncodersR","Time,RHipS,RKneS,RAnkS,RHipS_v,RKneS_v,RAnkS_v")
    ,PreRecorder("Pressure",kAdc_Header)
    ,PreRecOri("Pressure_ori",kAdc_Header)
    , LHipS_Enc(Encoder_L::HIP1), LKneS_Enc(Encoder_L::KNEE), LAnkS_Enc(Encoder_L::ANK1), RHipS_Enc(Encoder_R::HIP1), RKneS_Enc(Encoder_R::KNEE), RAnkS_Enc(Encoder_R::ANK1),adc0(0),adc1(1) //, LHipF_Enc(Encoder_L(1)), LAnkF_Enc(Encoder_L(4)), RHipF_Enc(Encoder_R(1)), RAnkF_Enc(Encoder_R(4))
    // ,filter_3_hz(FilterParam::Filter3Hz::a,FilterParam::Filter3Hz::b)
    ,digital_filter(FilterParam::Filter20Hz_2::a,FilterParam::Filter20Hz_2::b)
    ,left_enc_vel_filter(FilterParam::Filter45Hz_2::a,FilterParam::Filter45Hz_2::b)
    ,right_enc_vel_filter(FilterParam::Filter45Hz_2::a,FilterParam::Filter45Hz_2::b)
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
    int hip_s_pos = senHub.LHipS_Enc.ReadPos();
    int kne_s_pos = senHub.LKneS_Enc.ReadPos();
    int ank_s_pos = senHub.LAnkS_Enc.ReadPos();
    if(hip_s_pos>2048){
        hip_s_pos = hip_s_pos%2048-2048;
    }
    if(ank_s_pos>2048){
        ank_s_pos = ank_s_pos%2048-2048;
    }

    double hip_s_pos_f =  -1*hip_s_pos/4096.0*360; 
    double kne_s_pos_f = kne_s_pos/4096.0*360;
    double ank_s_pos_f = -1*ank_s_pos/4096.0*360;

    auto cur_enc_vel = senHub.left_enc_vel_filter.GetFilteredMea(std::array<double,3>{hip_s_pos_f- senHub.EncData[LHipS],kne_s_pos_f - senHub.EncData[LKneS],ank_s_pos_f - senHub.EncData[LAnkS]});
    senHub.EncVel[LHipS]=cur_enc_vel[0];
    senHub.EncVel[LKneS]=cur_enc_vel[1];
    senHub.EncVel[LAnkS]=cur_enc_vel[2];


    senHub.EncData[SensorHub::LHipS] = hip_s_pos_f; 
    senHub.EncData[SensorHub::LKneS] = kne_s_pos_f;
    senHub.EncData[SensorHub::LAnkS] = ank_s_pos_f;
    std::array<double,NUMENC> curMea{senHub.EncData[LHipS],senHub.EncData[LKneS],senHub.EncData[LAnkS],senHub.EncVel[LHipS],senHub.EncVel[LKneS],senHub.EncData[LAnkS]};
    // std::cout<<senHub.EncData[SensorHub::LHipS]<<','<<senHub.EncData[SensorHub::LKneS]<<','<<senHub.EncData[SensorHub::LAnkS]<<std::endl;
    // std::cout<<senHub.EncData[SensorHub::LHipS]<<std::endl;
    senHub.LEncRecorder.PushData(curMea);
    // std::cout<<"read\n";
    // std::cout<<senHub.EncData[SensorHub::LHipS]<<std::endl;
    // senHub.EncData[SensorHub::LHipF] = senHub.LAnkS_Enc.ReadPos();
    // senHub.EncData[SensorHub::LAnkF] = senHub.LAnkS_Enc.ReadPos();    
}
void SensorHub::UpdateREnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    int hip_s_pos = senHub.RHipS_Enc.ReadPos();
    int kne_s_pos = senHub.RKneS_Enc.ReadPos();
    int ank_s_pos = senHub.RAnkS_Enc.ReadPos();
    // int ank_s_pos_ori = ank_s_pos;

    if(hip_s_pos>2048){
        hip_s_pos = hip_s_pos%2048-2048;
    }
    if(ank_s_pos>2048){
        ank_s_pos = ank_s_pos%2048-2048;
    }

    double hip_s_pos_f = hip_s_pos/4096.0*360;
    double kne_s_pos_f = kne_s_pos/4096.0*360;
    double ank_s_pos_f = ank_s_pos/4096.0*360;
    //get velocity
    auto cur_enc_vel = senHub.right_enc_vel_filter.GetFilteredMea(std::array<double,3>{hip_s_pos_f- senHub.EncData[RHipS],kne_s_pos_f - senHub.EncData[RKneS],ank_s_pos_f - senHub.EncData[RAnkS]});
    senHub.EncVel[RHipS] = cur_enc_vel[0];
    senHub.EncVel[RKneS] = cur_enc_vel[1];
    senHub.EncVel[RAnkS] = cur_enc_vel[2];


    senHub.EncData[SensorHub::RHipS]=hip_s_pos_f;   ////TODO: read the correct encoder when encoders connected
    // // senHub.EncData[SensorHub::RHipF]=senHub.RKneS_Enc.ReadPos();
    senHub.EncData[SensorHub::RKneS]=kne_s_pos_f;
    senHub.EncData[SensorHub::RAnkS]=ank_s_pos_f;
    // // std::cout<<ank_s_pos_ori<<std::endl;
    // // senHub.EncData[SensorHub::RAnkF]=senHub.RKneS_Enc.ReadPos();
    // // std::cout<<senHub.EncData[SensorHub::RKneS]<<std::endl;
    // // std::cout<<senHub.EncData[SensorHub::RHipS]<<','<<senHub.EncData[SensorHub::RKneS]<<','<<senHub.EncData[SensorHub::RAnkS]<<std::endl;
    std::array<double,NUMENC> curMea{senHub.EncData[RHipS],senHub.EncData[RKneS],senHub.EncData[RAnkS],senHub.EncVel[RHipS],senHub.EncVel[RKneS],senHub.EncVel[RAnkS]};
    senHub.REncRecorder.PushData(curMea);
}
void SensorHub::UpdatePre()
{
    SensorHub & senHub = SensorHub::GetInstance();
    std::array<double,NUMPRE> avg_mea{0};
   
    for(int i=0;i<SensorHub::kAdcAvgWindow;i++){
        const std::array<double,8> &data = senHub.adc0.ReadData();
        const std::array<double,8> &data2 = senHub.adc1.ReadData();
        std::array<double,NUMPRE> cur_mea;
        std::memcpy(cur_mea.begin(),data.begin(),sizeof(double)*ADC::kDataLen);
        std::memcpy(cur_mea.begin()+8,data2.begin(),sizeof(double)*ADC::kDataLen);

        for(int i2=0;i2<NUMPRE;i2++){
            avg_mea[i2] = avg_mea[i2]+cur_mea[i2]/SensorHub::kAdcAvgWindow;
        }

    }
    // for(int i=0;i<NUMPRE;i++){
    //     std::cout<<avg_mea[i]<<',';
    // }
    // std::cout<<std::endl;
    
    // senHub.PreData = senHub.filter_3_hz.GetFilteredMea(cur_mea);
    senHub.PreData = senHub.digital_filter.GetFilteredMea(avg_mea);


    senHub.PreRecorder.PushData(senHub.PreData);

    //rec data before filtering
    senHub.PreRecOri.PushData(avg_mea);

   


    
   
  
}

