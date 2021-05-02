#ifndef TIMER_HPP
#define TIMER_HPP
#include<time.h>
#define NSEC_PER_SEC (1000000000) // The number of nsecs per sec.
#define NSEC 1
#define USEC (1000 * NSEC)
#define MSEC (1000 * USEC)
#define SEC (1000 * MSEC)
#define SAMPT 2000
class Timer
{
private:
    static void tsnorm(struct timespec *ts);
public:
    Timer(/* args */);
    ~Timer();
    
    static void Sleep(struct timespec *ts);
};


#endif //TIMER_HPP