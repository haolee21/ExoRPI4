#ifndef FILTER_PARAM_HPP
#define FILTER_PARAM_HPP
#include <array>
namespace FilterParam
{
    namespace Filter3Hz
    {
        const int Order = 4;
        const std::array<double,Order+1> b = {6.23869835e-05, 2.49547934e-04, 3.74321901e-04, 2.49547934e-04,6.23869835e-05};
        const std::array<double,Order+1> a = {1.        , -3.50778621,  4.64090241, -2.74265282,  0.61053481};

    }
    namespace Filter10Hz //It is actually 10 Hz, TODO: change it later
    {
        const int Order=2;
        const std::array<double,Order+1> b = {0.06745527, 0.13491055, 0.06745527};
        const std::array<double,Order+1> a = { 1.       , -1.1429805,  0.4128016};
    }

    namespace Filter20Hz_2
    {
        const int Order=2;
        const std::array<double,Order+1> b = {0.20657208, 0.41314417, 0.20657208};
        const std::array<double,Order+1> a = {1.        , -0.36952738,  0.19581571};
    }
    namespace Filter45Hz_2
    {
        const int Order=2;
        const std::array<double,Order+1> b= {0.8005924 , 1.60118481, 0.8005924};
        const std::array<double,Order+1> a = {1.        , 1.56101808, 0.64135154};
    }
    namespace Filter20Hz_5
    {
        const int Order = 5;
        const std::array<double,Order+1> b = {0.02193962, 0.1096981 , 0.21939621, 0.21939621, 0.1096981 ,0.02193962};
        const std::array<double,Order+1> a = {1.        , -0.98532524,  0.97384933, -0.38635656,  0.11116384, -0.01126351};
    }
    namespace Filter15Hz_5
    {
        const int Order = 5;
        const std::array<double,Order+1> b = {0.0069332 , 0.03466598, 0.06933196, 0.06933196, 0.03466598, 0.0069332};
        const std::array<double,Order+1> a = {1.        , -1.97590162,  2.01347303, -1.10261798,  0.32761833, -0.04070949};
    }
    
}
#endif