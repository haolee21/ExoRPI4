#ifndef CYLINDER_PARAM_HPP
#define CYLINDER_PARAM_HPP
#include <array>

#include "MPC_param.hpp"
namespace CylinderParam
{
    struct Params{
        std::array<std::array<float,MPC_STATE_NUM>,2> cl;
        std::array<std::array<float,MPC_STATE_NUM>,2> ch;
        double max_pos;
        double fri_coeff;
        double piston_area;
    };
    const Params kLTank = {
        MpcInitParam::kLTankCl,
        MpcInitParam::kLTankCh,
        1.0,
        0,
        0
    };
    const Params kLkne = {
        MpcInitParam::kLKneCl,
        MpcInitParam::kLKneCh,
        56739.5,
        0.0088964432, //0.001 in psi, 
        387.096 // unit: mm^2 0.6 in2
    };

    

}
#endif