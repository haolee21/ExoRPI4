#ifndef ADC_HPP
#define ADC_HPP


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
    

    static const short SEN1=1;
    static const short SEN2= 2;
    static const short SEN3= 3;
    static const short SEN4= 4;
    static const short SEN5= 5;
    static const short SEN6= 6;
    static const short SEN7= 7;
    static const short SEN8= 0;
    static const short DATALEN=8;
    const std::array<short,DATALEN>& ReadData();
private:
    // int *data;
    std::array<short,8> data;
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