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
    namespace Filter5Hz
    {
        const int Order=2;
        const std::array<float,Order+1> b = {0.02008337, 0.04016673, 0.02008337};
        const std::array<float,Order+1> a = {1.        , -1.56101808,  0.64135154};
    }
    
}
#endif