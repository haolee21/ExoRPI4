#ifndef RECORDER_HPP
#define RECORDER_HPP

#include<cstring>
#include<fstream>
#include<array>
#include<memory>
#include<future>
#include<sstream>
#include<iostream>
#include<functional>
#include<fstream>
#include<iterator>
#include"Timer.hpp"

#define REC_MAX_LEN 10000
template<class T,unsigned N>
class Recorder
{
private:
    const bool &data_rec_flag;
    const std::string &filePath;
    std::string fileName,header;
    std::array<std::array<T,N>,REC_MAX_LEN> tempData1;
    std::array<std::array<T,N>,REC_MAX_LEN> tempData2;
    std::array<std::array<T,N>,REC_MAX_LEN> *curData;

    std::array<unsigned,REC_MAX_LEN> rec_time1;
    std::array<unsigned,REC_MAX_LEN> rec_time2;
    std::array<unsigned,REC_MAX_LEN> *cur_rec_time;

    unsigned dataIdx;
    unsigned tempFileIdx;
    bool tempData_flag; //tempData_flag=true for tempData1, =false for tempData2
    std::future<void> saveCSV_future;
    static void SaveCSV(std::array<std::array<T,N>,REC_MAX_LEN> data,std::array<unsigned,REC_MAX_LEN> rec_time ,std::string filePath,std::string fileName,unsigned fileIdx,std::string header,unsigned endIdx=REC_MAX_LEN){

        std::string fullFileName = filePath+"/"+fileName + '_'+std::to_string(fileIdx)+".csv";

        std::ofstream writeCsv;
        std::ostringstream vts;
        writeCsv.open(fullFileName);
        vts<<header<<'\n';
        for(unsigned i=0;i<endIdx;i++){
            vts<<rec_time[i];
            vts<<',';

            std::array<T,N> curRow = data[i];
            //Note that I converted all save data into int, it is to avoid error when using uint8_t, it is equivalent to byte, thus, default will only output ascii 
            // if(sizeof(T)==sizeof(char))
            if((typeid(T)==typeid(u_int8_t)) || (typeid(T)==typeid(char)) )
            {
                 std::copy(curRow.begin(),curRow.end()-1,std::ostream_iterator<int>(vts,",")); //only -1 because this will not include the -1 member (1:3, only select 1,2)
            }
            else{
                 std::copy(curRow.begin(),curRow.end()-1,std::ostream_iterator<T>(vts,",")); //only -1 because this will not include the -1 member (1:3, only select 1,2)

            }

            if((typeid(T)==typeid(u_int8_t)) || (typeid(T)==typeid(char)) )
                vts<<(int)*(curRow.end()-1)<<'\n';//last element does not need ',' but '\n'
            else
                vts<<*(curRow.end()-1)<<'\n';//last element does not need ',' but '\n'
        }
        writeCsv<<vts.str();
        writeCsv.close();


        
    }
public:
    Recorder(std::string _fileName,std::string _header)
    :data_rec_flag(Timer::GetDataRec_flag()),filePath(Timer::GetFilePath()),fileName(_fileName),header(_header)
    {
        
        
        this->saveCSV_future = std::async(std::launch::async,[](){}); //This is for init the save file thread, so we don't need to generate a new thread
        

        this->dataIdx=0;
        this->tempFileIdx=0;
        this->tempData_flag=true;
        this->curData = &(this->tempData1);
        this->cur_rec_time = &(this->rec_time1);
        

    }
    ~Recorder(){
        this->saveCSV_future.wait();
        this->saveCSV_future.get();
        if(this->dataIdx!=0)
        Recorder::SaveCSV(*(this->curData),*(this->cur_rec_time),this->filePath,this->fileName,this->tempFileIdx,this->header,this->dataIdx);
        

        

    }
    
    void PushData(const std::array<T,N> data,int timer_offset=0){//timer offset is added if the rec value is not the current value, e.g., in MPC, gradient estimation requires control input, but we get the measurements at the beginning and get control at the end

        if(this->data_rec_flag){
            (*(this->cur_rec_time))[this->dataIdx]=Timer::GetCurTime()-timer_offset;
            (*(this->curData))[this->dataIdx++]=data;
            if(this->dataIdx==REC_MAX_LEN){
                this->saveCSV_future.wait();
                this->saveCSV_future.get();
                std::array<std::array<T,N>,REC_MAX_LEN> oldData = *(this->curData);
                std::array<unsigned,REC_MAX_LEN> old_rec_time = *(this->cur_rec_time);
                this->saveCSV_future = std::async(std::launch::async,[this,oldData,old_rec_time](){Recorder::SaveCSV(oldData,old_rec_time, this->filePath,this->fileName,this->tempFileIdx++,this->header);});//the reason I did not copy filePath etc is because these values will not change in single batch, but when reset, the filePath and all counters will change, so I use copy in reset
                if(this->tempData_flag){
                    this->curData=&(this->tempData2);
                    this->cur_rec_time = &(this->rec_time2);
                }
                else{
                    this->curData=&(this->tempData1);
                    this->cur_rec_time = &(this->rec_time1);
                }
                this->dataIdx=0;
                this->tempData_flag=!this->tempData_flag;
            }
        }
        else{
            
            this->Reset();
        }
    }
    void Reset(){
        if(this->dataIdx>0){
            this->saveCSV_future.wait();
            this->saveCSV_future.get();
            std::array<std::array<T,N>,REC_MAX_LEN> oldData = *(this->curData);
            std::array<unsigned,REC_MAX_LEN> old_rec_time = *(this->cur_rec_time);
            unsigned oldDataIdx=this->dataIdx;
            unsigned oldFileIdx = this->tempFileIdx++;
            std::string oldFilePath = this->filePath;
            this->saveCSV_future = std::async(std::launch::async,[this,oldData,old_rec_time,oldDataIdx,oldFilePath,oldFileIdx](){Recorder::SaveCSV(oldData,old_rec_time, oldFilePath,this->fileName,oldFileIdx,this->header,oldDataIdx);});   
            this->dataIdx=0;
            this->tempFileIdx=0;
            if(this->data_rec_flag){
                this->curData=&(this->tempData2);
                this->cur_rec_time = &(this->rec_time2);
            }
            else{
                this->curData=&(this->tempData1);
                this->cur_rec_time = &(this->rec_time1);
            }
        }
        
    }

    int ReturnRecIdx(){
        return this->dataIdx;
    }


};

#endif