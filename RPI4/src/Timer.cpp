#include "Timer.hpp"

Timer::Timer(std::function<void()>_tickFunction)
{
    this->tickFunction = _tickFunction;
    this->timeStamp =0;
}
Timer::~Timer()
{
}
void Timer::ClockTick(){
    this->timeStamp++;
    this->tickFunction();
}
unsigned Timer::GetCurTime(){
    return this->timeStamp;
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