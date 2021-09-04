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

class SensorHub
// this is a singleton class since it is directly link to the hardware, 
// config will not change unless I modify the hardware
{
public:
    
    ~SensorHub();
    const static int NUMENC = 10;    
    const static int NUMPRE = 8;
    static SensorHub& GetInstance();
    static const std::array<short,NUMENC>& GetEncData(); //I did not use lock here since they will be read-only arrays
    static const std::array<short,NUMPRE>& GetPreData(); //While data may not be sync, but it will be the most recent one
    
    SensorHub(const SensorHub&) = delete; // prevent copy singleton

    enum EncName{
        LHipS,LHipF,LKneS,LAnkS,LAnkF,
        RHipS,RHipF,RKneS,RAnkS,RAnkF
    };
    static void ResetEnc(SensorHub::EncName);
    static int Start(std::shared_ptr<Timer> baseTimer);//start the sensor update thread, since it is real-time, we will have to create the thread last and lock the memory
                                                       // a timer that is created in main is shared among SensorHub and ValveHub, I wish I have better way to do it
    static int Stop();
private:
    
    std::array<short,NUMENC> EncData;

    // Encoders, S is for sagittal plane, F is for frontal plane
    Encoder_L LHipS_Enc,LHipF_Enc,LKneS_Enc,LAnkS_Enc,LAnkF_Enc;
    Encoder_R RHipS_Enc,RHipF_Enc,RKneS_Enc,RAnkS_Enc,RAnkF_Enc;
    SensorHub();
    void ResetEncImpl(SensorHub::EncName);//the reset function is implement so I can avoid using a lot of SensorHub::Encorder since it is a member function

    // ADC 
    ADC adc0,adc1;


    //Sensor update
    // we are using a real-time thread for sensor update
    static void *SenUpdate(void*);
    bool senUpdate_flag;
    pthread_t rt_thread;
    std::shared_ptr<Timer> mainTimer;

    

    static void UpdateLEnc();//we will use std::async to launch sensor update tasks to improve speed
    static void UpdateREnc();
    static void UpdatePre();


};




#endif