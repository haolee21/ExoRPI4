#include "Encoder_L.hpp"
#include "Encoder_R.hpp"
#include <iostream>
#include <unistd.h>
#include <ADC.hpp>
#include <array>
int main()
{
    Encoder_L LAnk1(Encoder_L::ANK2);
    ADC adc1(0);
    for(int i=0;i<1000;i++){
        std::cout<<"loop"<<i<<"\n";
        std::cout<<"Pos: "<<LAnk1.ReadPos()<<std::endl;

        std::array<int,8> adcRes = adc1.ReadData();
        std::cout<<"adc: ";
        for(unsigned i2 =0;i2<adcRes.size();i2++){
            std::cout<<adcRes[i2]<<',';
        }
        std::cout<<std::endl;


        usleep(100000);

    }
    
}