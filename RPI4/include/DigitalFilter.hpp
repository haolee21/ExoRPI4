#ifndef DIGITAL_FILTER_HPP
#define DIGITAL_FILTER_HPP
#include <array>
#include <cstring>
template<class T,int M,int N>
// T: data type
// M: order of the filter
// N: number of each measurement
class DigitalFilter
{
private:
    std::array<std::array<T,N>,M> in_buf;
    std::array<std::array<T,N>,M> out_buf;
    int buf_idx;
    std::array<float,M+1> a;
    std::array<float,M+1> b;
    



public:
    DigitalFilter(std::array<float,M+1>_a,std::array<float,M+1>_b)
    :a(_a),b(_b)
    {
        this->buf_idx=0;

        std::array<T,N> zero_array;
        zero_array.fill(0);
        this->in_buf.fill(zero_array);
        this->out_buf.fill(zero_array);
 

    };
    ~DigitalFilter(){

    }
    std::array<T,N> GetFilteredMea(std::array<T,N> cur_mea){
        
        for(int idx=0;idx<N;idx++){
            T res_val = 0;//define in double to avoid overflow
            for(int im=1;im<M+1;im++){
                int pre_idx = (this->buf_idx+M-im)%M;
                res_val += this->b[im]*this->in_buf[pre_idx][idx]-this->a[im]*this->out_buf[pre_idx][idx];
            }
            res_val += this->b[0]*cur_mea[idx];
            this->out_buf[this->buf_idx][idx] = res_val;

        

        }
        this->in_buf[this->buf_idx] = cur_mea;
        std::array<T,N> res = out_buf[this->buf_idx];

        this->buf_idx++;
        if(this->buf_idx>=M){
            this->buf_idx=0;
        }


        return res;

    }
};




#endif