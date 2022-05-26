#ifndef MPC_PARAM_HPP
#define MPC_PARAM_HPP
#define MPC_STATE_NUM 11
#include <array>
namespace MpcInitParam
{
    const std::array<std::array<float,MPC_STATE_NUM>,2> kLTankCl
    {

        std::array<float,MPC_STATE_NUM>{-9.44900694,0.,717.06505177,0.,100.45015618,0.,4.86710724,0.,0.,23.49034113,76.12793007},
        std::array<float,MPC_STATE_NUM>{26.30185487,0.,-475.20096317,0.,-237.58392498,0.,-17.45066958,0.,0.,0.,-143.94265669}
    };
    const std::array<std::array<float,MPC_STATE_NUM>,2> kLTankCh
    {
        std::array<float,MPC_STATE_NUM>{30.35141617,0.,0.,0.,0.,0.,-91.04917334,0.,0.,0.,-332.95190565},
        std::array<float,MPC_STATE_NUM>{-49.6507161,0.,0.,0.,0.,0.,134.51951519,0.,-10.39619691,0.,342.41615334}
    };

    const std::array<std::array<float,MPC_STATE_NUM>,2> kLKneCl
    {
        std::array<float,MPC_STATE_NUM>{14.67798311,0.,0.,0.,0.,0.,-34.51145408,0.,0.,-132.11241483,-223.3002634},
        std::array<float,MPC_STATE_NUM>{-22.8866904, 0, 115.49115582, 0.,96.24270625,0.,2.73391824,0.,0.,527.21759316,0.}

    };
    
}
#endif