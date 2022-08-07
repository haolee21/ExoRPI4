#ifndef FILTER_PARAM_HPP
#define FILTER_PARAM_HPP
#include <array>
namespace FilterParam
{
    namespace Filter3Hz
    {
        const int Order = 4;
        const std::array<float,Order+1> b = {6.23869835e-05, 2.49547934e-04, 3.74321901e-04, 2.49547934e-04,6.23869835e-05};
        const std::array<float,Order+1> a = {1.        , -3.50778621,  4.64090241, -2.74265282,  0.61053481};

    }
    namespace Filter10Hz //It is actually 10 Hz, TODO: change it later
    {
        const int Order=2;
        const std::array<float,Order+1> b = {0.06745527, 0.13491055, 0.06745527};
        const std::array<float,Order+1> a = { 1.       , -1.1429805,  0.4128016};
    }

    namespace Filter20Hz 
    {
        const int Order=2;
        const std::array<float,Order+1> b = {0.20657208, 0.41314417, 0.20657208};
        const std::array<float,Order+1> a = {1.        , -0.36952738,  0.19581571};
    }
    
}
#endif