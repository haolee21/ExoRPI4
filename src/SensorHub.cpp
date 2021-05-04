#include "SensorHub.hpp"
#include <functional>


#include "Common.hpp"

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
const std::array<short, SensorHub::NUMENC> &SensorHub::GetEncData()
{
    return std::cref(SensorHub::GetInstance().EncData);
}
const std::array<short, SensorHub::NUMPRE> &SensorHub::GetPreData()
{
    return std::cref(SensorHub::GetInstance().adc0.ReadData());
}

SensorHub::~SensorHub()
{
    //we will have to release all the memory we locked in Start();
    munlockall();
}
SensorHub::SensorHub() //initialize member in list since Encoder has no default constructor
    : LHipS_Enc(Encoder_L(0)), LHipF_Enc(Encoder_L(1)), LKneS_Enc(Encoder_L(2)), LAnkS_Enc(Encoder_L(3)), LAnkF_Enc(Encoder_L(4)), RHipS_Enc(Encoder_R(0)), RHipF_Enc(Encoder_R(1)), RKneS_Enc(Encoder_R(2)), RAnkS_Enc(Encoder_R(3)), RAnkF_Enc(Encoder_R(4)), adc0(ADC(0))
{
    this->senUpdate_flag = false;
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

void *SensorHub::SenUpdate(void *data)
{
    std::cout << "update thread starts\n";
    // SensorHub &senHub =  SensorHub::GetInstance();
    struct timespec t;
    long int interval = SAMPT * USEC;

    // int loopCount = 0;
    // int64_t cur_last = 0;
    // std::array<int64_t, 100000> diffTime;
    // auto start = std::chrono::high_resolution_clock::now();


    //we do a dry run of updating sensor here, can somehow decrease the later loop time
    std::future<void> update1 = std::async(std::launch::async, SensorHub::UpdateLEnc);
    std::future<void> update2 = std::async(std::launch::async, SensorHub::UpdateREnc);
    update1.wait();
    update2.wait();
    update1.get();
    update2.get();
    // std::chrono::nanoseconds elapsed;
    // int64_t duration;



    clock_gettime(CLOCK_MONOTONIC, &t);
    // ref to https://linux.die.net/man/2/clock_gettime, maybe can improve stability
    // clock_gettime(CLOCK_BOOTTIME,&t);
    while (SensorHub::GetInstance().senUpdate_flag)
    {

        update1 = std::async(std::launch::async, SensorHub::UpdateLEnc); //the efficiency increase dramatically, I am not sure why it can make such difference
        update2 = std::async(std::launch::async, SensorHub::UpdateREnc);

        t.tv_nsec += interval;

        // elapsed = std::chrono::high_resolution_clock::now() - start;
        // duration = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        // if (loopCount < diffTime.size())
        // {
        //     diffTime[loopCount] = duration - cur_last;
        //     cur_last = duration;
        // }

        Timer::Sleep(&t);
        // loopCount++;
        update1.wait();
        update2.wait();
        update1.get();
        update2.get();
    }

    // elapsed = std::chrono::high_resolution_clock::now() - start;
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    // std::cout << "time diff: ";
    // for (int i = 0; i < loopCount; i++)
    // {
    //     std::cout << diffTime[i] - SAMPT << ',';
    // }
    // std::cout << "avg samp time(us): " << duration / loopCount << std::endl;
}

void SensorHub::UpdateLEnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    senHub.EncData[0] = senHub.LHipS_Enc.ReadPos();
    senHub.EncData[1] = senHub.LHipS_Enc.ReadPos();
    senHub.EncData[2] = senHub.LHipS_Enc.ReadPos();
    senHub.EncData[3] = senHub.LHipS_Enc.ReadPos();
    senHub.EncData[4] = senHub.LHipS_Enc.ReadPos();
}
void SensorHub::UpdateREnc()
{
    SensorHub &senHub = SensorHub::GetInstance();
    // senHub.EncData[5]=senHub.LHipS_Enc.ReadPos();
    // senHub.EncData[6]=senHub.LHipS_Enc.ReadPos();
    // senHub.EncData[7]=senHub.LHipS_Enc.ReadPos();
    // senHub.EncData[8]=senHub.LHipS_Enc.ReadPos();
    // senHub.EncData[9]=senHub.LHipS_Enc.ReadPos();
}

int SensorHub::Start()
{ //ref from https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base

    SensorHub::GetInstance().senUpdate_flag = true;

    RT::Init();
    int ret = RT::StartThread(SensorHub::GetInstance().rt_thread, SensorHub::SenUpdate, 80);
    return ret;
}

int SensorHub::Stop()
{
    SensorHub::GetInstance().senUpdate_flag = false;
    /* Join the thread and wait until it is done */
    int ret = pthread_join(SensorHub::GetInstance().rt_thread, NULL);
    // if (ret)
    //     printf("join pthread failed: %m\n");
    //print the line outside
    return ret;
}