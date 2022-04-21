#include "Timer.hpp"
bool Timer::dataRec_flag=false; //it has to be false defautly, will be a const ref to all class that use
std::string Timer::filePath="";
namespace fs = boost::filesystem;
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
    std::array<float,120*1000> timeDiff; //can only run around 120 sec
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
        


        //update actuation
        for(unsigned i=0;i<timer.conCallbacks.size();i++){
            timer.conFutures[i]=std::async(std::launch::async,timer.conCallbacks[i]);
        }
        for(unsigned i=0;i<timer.conCallbacks.size();i++){
            timer.conFutures[i].wait();
        }
        for(unsigned i=0;i<timer.conCallbacks.size();i++){
            timer.conFutures[i].get();
        }
        // Valves_hub::UpdateValve();


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
const bool& Timer::GetDataRec_flag(){
    return std::ref(Timer::dataRec_flag);
}
bool Timer::StartRec(){
    
    if(Timer::dataRec_flag){
        Timer::EndRec();
       

    }
    fs::path data_dir(fs::current_path());
    std::string homeFolder = data_dir.string();
    std::string filePath;
    {
        //create the directory to save the data with datetime as folder name
        using namespace std;
        time_t result = time(nullptr);
        tm* timePtr = localtime(&result);
        std::stringstream curDate;
        curDate<<timePtr->tm_year+1900<<'-'<<std::setw(2)<<std::setfill('0')<<timePtr->tm_mon+1<<setw(2)<<setfill('0')<<timePtr->tm_mday<<'-'<<setw(2)<<setfill('0')<<timePtr->tm_hour<<setw(2)<<setfill('0')<<timePtr->tm_min<<'-'<<setw(2)<<setfill('0')<<timePtr->tm_sec;
        filePath = homeFolder +'/'+ curDate.str();
    }
    std::cout<<"folder name: "<<filePath<<std::endl;
    if (fs::is_directory(filePath)){
        throw std::invalid_argument("Data folder already exists\n");
        return false;
    }
    else{
        std::cout<<"SYS:TIMER:create new directory to save rec\n";
        fs::create_directory(filePath);
        Timer::filePath = filePath;
        Timer::dataRec_flag = true;
        return true;
    }
}
void Timer::EndRec(){
    Timer::dataRec_flag=false;
}
const std::string& Timer::GetFilePath(){
    return std::ref(Timer::filePath);
}
