#include "SensorHub.hpp"
#include <functional>

// crete rt tasks
#include <limits.h>

#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

//for timer

#include <malloc.h>
#include <sys/resource.h> // needed for getrusage
#define MEMSIZE (100*1024) // 100kB
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
    return std::cref(SensorHub::GetInstance().PreData);
}

SensorHub::~SensorHub()
{
    //we will have to release all the memory we locked in Start();
    munlockall();
}
SensorHub::SensorHub() //initialize member in list since Encoder has no default constructor
    : LHipS_Enc(Encoder_L(0)), LHipF_Enc(Encoder_L(1)), LKneS_Enc(Encoder_L(2)), LAnkS_Enc(Encoder_L(3)), LAnkF_Enc(Encoder_L(4))
    , RHipS_Enc(Encoder_R(0)), RHipF_Enc(Encoder_R(1)), RKneS_Enc(Encoder_R(2)), RAnkS_Enc(Encoder_R(3)), RAnkF_Enc(Encoder_R(4))
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
    std::cout<<"update thread starts\n";
    SensorHub &senHub =  SensorHub::GetInstance();
    struct timespec t;
    long int interval = SAMPT* USEC;
    clock_gettime(CLOCK_MONOTONIC, &t);
    while(SensorHub::GetInstance().senUpdate_flag){
        
        senHub.EncData[0]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[1]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[2]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[3]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[4]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[5]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[6]= senHub.LHipF_Enc.ReadPos();
        senHub.EncData[7]= senHub.LHipF_Enc.ReadPos();

        
        


        t.tv_nsec+=interval;
        Timer::Sleep(&t);
    }


}

int SensorHub::Start()
{ //ref from https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/application_base

    SensorHub::GetInstance().senUpdate_flag=true;
    struct sched_param param;
    pthread_attr_t attr;
   
    int ret;
    // lock memory
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("mlockall failed: %m\n");
        exit(-2);
    }
    // force page fault to make sure all page are locked on memory
    // ref: https://rt.wiki.kernel.org/index.php/Simple_memory_locking_example
    struct rusage usage;
    int page_size = sysconf(_SC_PAGESIZE);
    char *buffer = (char*)malloc(MEMSIZE);
    int i;
    for(i=0;i<MEMSIZE;i+=page_size){
        buffer[i] = 0;
        getrusage(RUSAGE_SELF, &usage);
        printf("Major-pagefaults:%ld, Minor Pagefaults:%ld\n", usage.ru_majflt, usage.ru_minflt);
    }
    free(buffer); //free the buffer, but due to mlockall, we will have MEMSIZE memory won't cause page fault

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        goto out;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        goto out;
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        goto out;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&SensorHub::GetInstance().rt_thread, &attr, SensorHub::SenUpdate,NULL);
    if (ret)
    {
        printf("create pthread failed\n");
        goto out;
    }



out:
    return ret;
}

int SensorHub::Stop(){
    SensorHub::GetInstance().senUpdate_flag=false;
    /* Join the thread and wait until it is done */
    int ret = pthread_join(SensorHub::GetInstance().rt_thread, NULL);
    // if (ret)
    //     printf("join pthread failed: %m\n");
    //print the line outside
    return ret;

}