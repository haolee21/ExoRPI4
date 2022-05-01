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

#define DEG (360.0f/4096.0f)

class SensorHub
// this is a singleton class since it is directly link to the hardware, 
// config will not change unless I modify the hardware
{
public:
    
    ~SensorHub();
    const static int NUMENC = 6;    
    const static int NUMPRE = 8; //always 8 since ADC has 8 channels
    
    const float ENC_DEN = 4096.0;
    const float PRE_DEN = 65536;

    static SensorHub& GetInstance();
    static const std::array<u_int16_t,NUMENC>& GetEncData(); //I did not use lock here since they will be read-only arrays
    static const std::array<u_int16_t,NUMPRE>& GetPreData(); //While data may not be sync, but it will be the most recent one
    
    SensorHub(const SensorHub&) = delete; // prevent copy singleton


    // data index
    // we use enum for encoder since the order can be determined by ourself (read ENC order)
    enum EncName{
        LHipS,LKneS,LAnkS,//,LHipF,LAnkF
        RHipS,RKneS,RAnkS//RHipF,RAnkF
    };
    // Pressure sensor index need to be assigned since it is determine by ADC channels
    static const u_int8_t LKneP=0,RKneP=1,LAnkP=2,RAnkP=3,TankP=4;//TODO: check the order before use


    static void ResetEnc(SensorHub::EncName);
    static void UpdateLEnc();//we will use std::async to launch sensor update tasks to improve speed
    static void UpdateREnc();
    static void UpdatePre(); //we don't this now since updating pressure sensor only need to read adc with SPI

                       
    
private:
    
    std::array<u_int16_t,NUMENC> EncData;
    std::array<u_int16_t,NUMPRE> PreData;
    Recorder<uint16_t,NUMENC/2> LEncRecorder;
    Recorder<uint16_t,NUMENC/2> REncRecorder;
    Recorder<uint16_t,NUMPRE> PreRecorder;

    // Encoders, S is for sagittal plane, F is for frontal plane
    Encoder_L LHipS_Enc,LKneS_Enc,LAnkS_Enc; //LHipF_Enc,LAnkF_Enc
    Encoder_R RHipS_Enc,RKneS_Enc,RAnkS_Enc;//,RHipF_Enc,RAnkF_Enc
    SensorHub();
    void ResetEncImpl(SensorHub::EncName);//the reset function is implement so I can avoid using a lot of SensorHub::Encorder since it is a member function

    // ADC 
    ADC adc0;


    
    

    

    

};

#endif