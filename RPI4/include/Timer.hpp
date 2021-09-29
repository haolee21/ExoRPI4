#ifndef TIMER_HPP
#define TIMER_HPP
#include<time.h>
#define NSEC_PER_SEC (1000000000) // The number of nsecs per sec.
#define NSEC 1
#define USEC (1000 * NSEC)
#define MSEC (1000 * USEC)
#define SEC (1000 * MSEC)
#define SAMPT 1250

#include <functional>
#include <pthread.h>
#include "RT.hpp"
#include <future>
#include <vector>
#include <iostream>
class Timer
{
    //this class has mix usage. Static functions are for real-time clocks nano sleep or number rounding
    //member functions are timer for both SensorHub and ValveHub
    public:
    Timer();
    ~Timer();
    
    static unsigned GetCurTime();
    static void Sleep(struct timespec *ts);
    static int StartRT();
    static int StopRT();

    static void Add_senCallback(std::function<void()> fun);
    static void Add_conCallback(std::function<void()> fun);
    static std::function<void()> senUpdateFun;
    static std::function<void()> controlUpdateFun;
private:
    // sensor/controller update functions, will ran in timer tick function
    static std::vector<std::function<void()>> senCallbacks;
    static std::vector<std::function<void()>> conCallbacks;
    
    static std::vector<std::future<void>> senFutures;
    static std::vector<std::future<void>> conFutures;
    static pthread_t rt_thread;
    static bool updateFlag;
    static void* TimerTick(void*);

    static unsigned timeStamp; //the time scale will be 1/sampFreq

    //below static functions are for real-time nano clocks     
    static void tsnorm(struct timespec *ts);
    

    
};


#endif //TIMER_HPP