#ifndef TIMER_HPP
#define TIMER_HPP
#include<time.h>
#define NSEC_PER_SEC (1000000000) // The number of nsecs per sec.
#define NSEC 1
#define USEC (1000 * NSEC)
#define MSEC (1000 * USEC)
#define SEC (1000 * MSEC)
#define SAMPT 1000  //sampling period in uS, make sure 

#include <functional>
#include <pthread.h>
#include "RT.hpp"
#include <future>
#include <vector>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <cstring>
#include <sstream>
#include <iomanip>
// #include "Valves_hub.hpp"
class Timer
{
    //this class has mix usage. Static functions are for real-time clocks nano sleep or number rounding
    //member functions are timer for both SensorHub and ValveHub
    public:
    
    ~Timer();
    Timer(const Timer&)=delete;
    static Timer& GetInstance();

    
    static unsigned GetCurTime();
    static void Sleep(struct timespec *ts);
    static int StartRT();
    static int StopRT();

    static void Add_senCallback(std::function<void()> fun);
    static void Add_conCallback(std::function<void()> fun);
    
    static std::function<void()> senUpdateFun;
    static std::function<void()> controlUpdateFun;


    // Data recording functions, it is binded to the timer because the "directory name" is based on the current time
    const static bool& GetDataRec_flag(); 
    const static std::string& GetFilePath();
    static bool StartRec();//create a new directory and sync time when start recording
    static void EndRec();
    
    
private:
    Timer();
    // sensor/controller update functions, will ran in timer tick function
    std::vector<std::function<void()>> senCallbacks;
    std::vector<std::function<void()>> conCallbacks;
    
    
    std::vector<std::future<void>> senFutures;
    std::vector<std::future<void>> conFutures;
    pthread_t rt_thread;
    bool updateFlag;
    static void* TimerTick(void*);

    unsigned timeStamp; //the time scale will be 1/sampFreq

    //below static functions are for real-time nano clocks     
    static void tsnorm(struct timespec *ts);
    
    //Data recording private members
    static bool dataRec_flag;
    static std::string filePath;

    
};


#endif //TIMER_HPP