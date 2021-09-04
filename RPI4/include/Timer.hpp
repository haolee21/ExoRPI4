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
class Timer
{
    //this class has mix usage. Static functions are for real-time clocks nano sleep or number rounding
    //member functions are timer for both SensorHub and ValveHub
private:
    std::function<void()> tickFunction;
    unsigned timeStamp; //the time scale will be 1/sampFreq

    //below static functions are for real-time nano clocks     
    static void tsnorm(struct timespec *ts);
    
public:
    Timer(std::function<void()> tikFunction);
    ~Timer();
    void ClockTick();
    unsigned GetCurTime();
    static void Sleep(struct timespec *ts);
    
};


#endif //TIMER_HPP