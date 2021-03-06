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
    unsigned dataIdx;
    unsigned tempFileIdx;
    bool tempData_flag; //tempData_flag=true for tempData1, =false for tempData2
    std::future<void> saveCSV_future;
    static void SaveCSV(std::array<std::array<T,N>,REC_MAX_LEN> data,std::string filePath,std::string fileName,unsigned fileIdx,std::string header,unsigned endIdx=REC_MAX_LEN){

        std::string fullFileName = filePath+"/"+fileName + '_'+std::to_string(fileIdx)+".csv";

        std::ofstream writeCsv;
        std::ostringstream vts;
        writeCsv.open(fullFileName);
        vts<<header<<'\n';
        for(unsigned i=0;i<endIdx;i++){
            std::array<T,N> curRow = data[i];
            //Note that I converted all save data into int, it is to avoid error when using uint8_t, it is equivalent to byte, thus, default will only output ascii 
            std::copy(curRow.begin(),curRow.end()-1,std::ostream_iterator<int>(vts,",")); //only -1 because this will not include the -1 member (1:3, only select 1,2)


            vts<<(int)*(curRow.end()-1)<<'\n';//last element does not need ',' but '\n'
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
        

    }
    ~Recorder(){
        this->saveCSV_future.wait();
        this->saveCSV_future.get();
        if(this->dataIdx!=0)
        Recorder::SaveCSV(*(this->curData),this->filePath,this->fileName,this->tempFileIdx,this->header,this->dataIdx);
        

        

    }
    
    void PushData(std::array<T,N> data){

        if(this->data_rec_flag){
            (*(this->curData))[this->dataIdx++]=data;
            if(this->dataIdx==REC_MAX_LEN){
                this->saveCSV_future.wait();
                this->saveCSV_future.get();
                std::array<std::array<T,N>,REC_MAX_LEN> oldData = *(this->curData);
                this->saveCSV_future = std::async(std::launch::async,[this,oldData](){Recorder::SaveCSV(oldData,this->filePath,this->fileName,this->tempFileIdx++,this->header);});//the reason I did not copy filePath etc is because these values will not change in single batch, but when reset, the filePath and all counters will change, so I use copy in reset
                if(this->tempData_flag){
                    this->curData=&(this->tempData2);
                }
                else{
                    this->curData=&(this->tempData1);
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
            unsigned oldDataIdx=this->dataIdx;
            unsigned oldFileIdx = this->tempFileIdx++;
            std::string oldFilePath = this->filePath;
            this->saveCSV_future = std::async(std::launch::async,[this,oldData,oldDataIdx,oldFilePath,oldFileIdx](){Recorder::SaveCSV(oldData,oldFilePath,this->fileName,oldFileIdx,this->header,oldDataIdx);});   
            this->dataIdx=0;
            this->tempFileIdx=0;
            if(this->data_rec_flag){
                this->curData=&(this->tempData2);
            }
            else{
                this->curData=&(this->tempData1);
            }
        }
        
    }


};

#endif