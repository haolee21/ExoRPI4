#ifndef SENSORHUB_HPP
#define SENSORHUB_HPP
#include <array>
#include <cstring>
#include <iostream>
#include "Encoder_L.hpp"
#include "Encoder_R.hpp"
#include <pthread.h>
#include "Timer.hpp"
#include "ADC.hpp"
#include <chrono>
#include <memory>
#include <Timer.hpp>
#include "Recorder.hpp"
#include "FilterParam.hpp"
#include "DigitalFilter.hpp"

#define DEG (360.0f/4096.0f)
#define ADC_NAME "Time"

class SensorHub
// this is a singleton class since it is directly link to the hardware, 
// config will not change unless I modify the hardware
{
private:
static const unsigned kPreSen1=0,kPreSen2=1,kPreSen3=2,kPreSen4=3,kPreSen5=4,kPreSen6=5,kPreSen7=6,kPreSen8=7;
static const unsigned kPreSen9=8,kPreSen10=9,kPreSen11=10,kPreSen12=11,kPreSen13=12,kPreSen14=13,kPreSen15=14,kPreSen16=15;

public:
    
    ~SensorHub();
    constexpr static int NUMENC=6;
    constexpr static int NUMPRE = 16;//always 16 since 2 ADC has 8 channels
    constexpr static double ENC_DEN = 4096.0;
    constexpr static double PRE_DEN = 65536;

    static SensorHub& GetInstance();
    static const std::array<double,NUMENC>& GetEncData(); //I did not use lock here since they will be read-only arrays
    static const std::array<double,NUMENC>& GetEncDiff();
    static const std::array<double,NUMPRE>& GetPreData(); //While data may not be sync, but it will be the most recent one
    // static const std::array<double,NUMPRE>& GetPreFiltered(); //get the filtered pressure reading

    SensorHub(const SensorHub&) = delete; // prevent copy singleton

    
    // data index
    // we use enum for encoder since the order can be determined by ourself (read ENC order)
    enum EncName{
        LHipS,LKneS,LAnkS,//,LHipF,LAnkF
        RHipS,RKneS,RAnkS//RHipF,RAnkF
    };
    enum class AdcName{ //The order of this enum must follow the order on the pcb board
        
        Tank = kPreSen14,
        RTank = kPreSen9,
        LTank = kPreSen12,
        LKneFLex=kPreSen10,
        RKneFlex = kPreSen11,
        RKneExt = kPreSen16,
        RAnkExt = kPreSen15,
        LKneExt = kPreSen2,
        LAnkExt = kPreSen1, //TODO: check it whenever you reconnect the pressure sensors
        LAnkFlex = kPreSen5,
        RAnkFlex = kPreSen6,
    };
    const std::string kAdc_Header = "Time,LAnkPla,LKneExt,na,na,LAnkDor,RAnkDor,na,na,RTank,LKneFlex,RKneFlex,LTank,na,Tank,RAnkPla,RKneExt";
    
    // Pressure sensor index need to be assigned since it is determine by ADC channels
    


    static void ResetEnc(SensorHub::EncName);
    static void UpdateLEnc();//we will use std::async to launch sensor update tasks to improve speed
    static void UpdateREnc();
    static void UpdatePre(); //we don't this now since updating pressure sensor only need to read adc with SPI

                       
    
private:
    
    std::array<double,NUMENC> EncData;
    std::array<double,NUMENC> EncDiff;
    std::array<double,NUMPRE> PreData;
    Recorder<double,NUMENC> LEncRecorder; //pos+vel
    Recorder<double,NUMENC> REncRecorder;

    Recorder<double,NUMPRE> PreRecorder;
    Recorder<double,NUMPRE> PreRecOri;

    // Encoders, S is for sagittal plane, F is for frontal plane
    Encoder_L LHipS_Enc,LKneS_Enc,LAnkS_Enc; //LHipF_Enc,LAnkF_Enc
    Encoder_R RHipS_Enc,RKneS_Enc,RAnkS_Enc;//,RHipF_Enc,RAnkF_Enc
    SensorHub();
    void ResetEncImpl(SensorHub::EncName);//the reset function is implement so I can avoid using a lot of SensorHub::Encorder since it is a member function

    // ADC 
    ADC adc0,adc1;
    static const int kAdcAvgWindow=10;

    //Butterworth filter for ADC
    // DigitalFilter<double,FilterParam::Filter3Hz::Order,NUMPRE> filter_3_hz;
    DigitalFilter<double,FilterParam::Filter20Hz_2::Order,NUMPRE> digital_filter;
    DigitalFilter<double,FilterParam::Filter5Hz_2::Order,NUMENC/2> left_enc_vel_filter;
    DigitalFilter<double,FilterParam::Filter5Hz_2::Order,NUMENC/2> right_enc_vel_filter;

   


    

    

};

#endif