#include "Timer.hpp"

Timer::Timer(/* args */)
{
}

Timer::~Timer()
{
}
void Timer::tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC)
    {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}
void Timer::Sleep(struct timespec *ts){
    Timer::tsnorm(ts);
    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,ts,NULL);

}