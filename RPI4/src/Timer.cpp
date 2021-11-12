#include "Timer.hpp"

Timer::Timer()
{
    
}
Timer::~Timer()
{
}
Timer& Timer::GetInstance(){
    static Timer instance;
    return instance;
}





unsigned Timer::GetCurTime(){
    
    return Timer::GetInstance().timeStamp;
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
    Timer::GetInstance().timeStamp=0;
    Timer::GetInstance().updateFlag = true;
    int ret = RT::StartThread(Timer::GetInstance().rt_thread,Timer::TimerTick,RT::RT_PRIORITY);
    return ret;
}
int Timer::StopRT(){
    Timer::GetInstance().updateFlag=false;
    int ret = pthread_join(Timer::GetInstance().rt_thread,NULL);
    return ret;
}

void* Timer::TimerTick(void*){
    std::cout<<"RT Task Start\n";
    struct timespec t;
    long int interval = SAMPT*USEC;

    //////////////////////////////////////////////////////////
    unsigned timeDiff_idx=0; //TODO: testing loop period only, should be commented in final version
    std::array<float,60*1000> timeDiff; //can only run around 60 sec
    auto t_start=std::chrono::high_resolution_clock::now();
    auto t_end = std::chrono::high_resolution_clock::now();
    ///////////////////////////////////////////////////////////

    Timer &timer = Timer::GetInstance();
    clock_gettime(CLOCK_MONOTONIC,&t);
    while(timer.updateFlag){
        //update sensor 
        for(unsigned i=0;i<timer.senCallbacks.size();i++){
            timer.senFutures[i]=std::async(std::launch::async,timer.senCallbacks[i]);
        }
        t.tv_nsec+=interval;  //TODO: finish this, need to think about how to make it efficient
        Timer::Sleep(&t);

        ///////////////////////////////////////////////// TODO: testing loop period only
        t_end = std::chrono::high_resolution_clock::now();
        timeDiff[timeDiff_idx++]=std::chrono::duration<float,std::micro>(t_end-t_start).count();
        t_start = t_end;
        /////////////////////////////////////////////////


        for(unsigned i=0;i<timer.senCallbacks.size();i++){
            timer.senFutures[i].wait();
            // timer.senFutures[i].get();
        }
        for(unsigned i=0;i<timer.senCallbacks.size();i++){
            timer.senFutures[i].get();
        }
        
        //update valve
        Valves_hub::UpdateValve();


        timer.timeStamp++;
    }
    /////////////////////////////////////////// TODO: testing loop period only
    std::cout<<"Time diff: \n";
    for(unsigned i=0;i<timeDiff_idx;i++){
        std::cout<<timeDiff[i]<<',';
    }
    ////////////////////////////////////////////

    
    return 0;
}
void Timer::Add_senCallback(std::function<void()> fun){
    Timer& timer=Timer::GetInstance();

    timer.senCallbacks.push_back(fun);
    timer.senFutures.push_back(std::async(std::launch::async,fun));
    timer.senFutures.back().wait();
    timer.senFutures.back().get();
}
void Timer::Add_conCallback(std::function<void()> fun){
    Timer& timer=Timer::GetInstance();
    timer.conCallbacks.push_back(fun);
    timer.conFutures.push_back(std::async(std::launch::async,fun));
    timer.conFutures.back().wait();
    timer.conFutures.back().get();
}