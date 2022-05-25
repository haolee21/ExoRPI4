#ifndef MPC_PARAM_HPP
#define MPC_PARAM_HPP
#include <array>
namespace MpcInitParam
{
    const std::array<std::array<float,13>,2> kLTankCl
    {
        std::array<float,13> {0.0,0.0,-2.55414539,0.0,828.30411474,0.0,64.49552201,0.0,1.42154453,0.0,91.03587057,36.73168344,0.0},
        std::array<float,13> {0.0,0.0,21.96427581,0.0,-678.69509286,0.0,-210.04224453,0.0,-12.87055314,0.0,0.0,0.0,-127.30619994}
    };
    const std::array<std::array<float,13>,2> kLTankCh
    {
        std::array<float,13>{0.,0.,17.6077691,0.,0.,0.,0.,0.,-64.48492499,0.,0.,0.,-334.3004319},
        std::array<float,13>{0.,0.,-46.0709009,0.,0.,0.,0.,0.,128.80706857,0.,0.,0.,346.2909906}
    };
}
#endif