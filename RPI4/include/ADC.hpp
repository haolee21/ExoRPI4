#ifndef ADC_HPP
#define ADC_HPP


#include <string>
#include <cstring>
#include "Pin.hpp"
#include <memory>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <array>

class ADC
{
public:
    ADC(short adc_idx);
    ~ADC();
    

    static const uint8_t SEN0= 1;
    static const uint8_t SEN1= 2;
    static const uint8_t SEN2= 3;
    static const uint8_t SEN3= 4;
    static const uint8_t SEN4= 5;
    static const uint8_t SEN5= 6;
    static const uint8_t SEN6= 7;
    static const uint8_t SEN7= 0;
    static const uint8_t DATALEN=8;
    const std::array<u_int16_t,DATALEN>& ReadData();
private:
    // int *data;
    std::array<u_int16_t,8> data;
    void startConv();


    static const Pin conv;
    static const Pin reset;
    static const Pin eoc;



    void init_ADC();
    int fd;
    char _spiTxRx(unsigned int len);

    char ch_list[8];

    char rxBuf[100];
    char txBuf[100];


};


#endif //ADC_HPP