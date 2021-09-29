#include "Timer.hpp"

Timer::Timer()
{
    
}
Timer::~Timer()
{
}

bool Timer::updateFlag = false;
std::vector<std::function<void()>> Timer::senCallbacks;
std::vector<std::function<void()>> Timer::conCallbacks;
std::vector<std::future<void>> Timer::senFutures;
std::vector<std::future<void>> Timer::conFutures;
pthread_t Timer::rt_thread;
unsigned Timer::timeStamp;

unsigned Timer::GetCurTime(){
    return Timer::timeStamp;
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
int Timer::StartRT(){
    RT::Init();
    Timer::timeStamp=0;
    Timer::updateFlag = true;
    int ret = RT::StartThread(Timer::rt_thread,Timer::TimerTick,RT::RT_PRIORITY);
    return ret;
}
int Timer::StopRT(){
    Timer::updateFlag=false;
    int ret = pthread_join(Timer::rt_thread,NULL);
    return ret;
}

void* Timer::TimerTick(void*){
    std::cout<<"RT Task Start\n";
    struct timespec t;
    long int interval = SAMPT*USEC;
    clock_gettime(CLOCK_MONOTONIC,&t);
    while(Timer::updateFlag){
        //update sensor 
        for(unsigned i=0;i<Timer::senCallbacks.size();i++){
            Timer::senFutures[i]=std::async(std::launch::async,Timer::senCallbacks[i]);
        }
        t.tv_nsec+=interval;  //TODO: finish this, need to think about how to make it efficient
        Timer::Sleep(&t);
        for(unsigned i=0;i<Timer::senCallbacks.size();i++){
            Timer::senFutures[i].wait();
        }
        for(unsigned i=0;i<Timer::senCallbacks.size();i++){
            Timer::senFutures[i].get();
        }
        Timer::timeStamp++;
    }
}
void Timer::Add_senCallback(std::function<void()> fun){
    
    Timer::senCallbacks.push_back(fun);
    Timer::senFutures.push_back(std::async(std::launch::async,fun));
    Timer::senFutures.back().wait();
    Timer::senFutures.back().get();
}
void Timer::Add_conCallback(std::function<void()> fun){
    Timer::conCallbacks.push_back(fun);
    Timer::conFutures.push_back(std::async(std::launch::async,fun));
    Timer::conFutures.back().wait();
    Timer::conFutures.back().get();
}