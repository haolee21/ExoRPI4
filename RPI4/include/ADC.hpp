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
    

    
    static const uint8_t kDataLen=8;
    const std::array<double,kDataLen>& ReadData();
private:
    // int *data;
    std::array<double,8> data;
    void startConv();


    static const Pin conv;
    static const Pin reset;
    static const Pin eoc;



    void init_ADC();
    int fd;
    char _spiTxRx(unsigned int len);

    char ch_list[8];

    char rxBuf[30];
    char txBuf[30];


};


#endif //ADC_HPP