#ifndef CYLINDER_PARAM_HPP
#define CYLINDER_PARAM_HPP
#include <array>

#include "MPC_param.hpp"
namespace PneumaticParam
{

    struct CylinderParam
    {
        std::array<std::array<double, MPC_STATE_NUM>, 2> cl_ext;
        std::array<std::array<double, MPC_STATE_NUM>, 2> ch_ext;
        std::array<std::array<double, MPC_STATE_NUM>, 2> cl_flex;
        std::array<std::array<double, MPC_STATE_NUM>, 2> ch_flex;
        double max_pos;
        double fri_coeff;
        double piston_area_ext;
        double piston_area_flex;
    };
    struct ReservoirParam
    {
        std::array<std::array<double, MPC_STATE_NUM>, 2> cl;
        std::array<std::array<double, MPC_STATE_NUM>, 2> ch;
    };

    const CylinderParam kLKne = {

        MpcInitParam::kLKneCl,
        MpcInitParam::kLKneCh,
        MpcInitParam::kLKneCl, //TODO: update it with the real param
        MpcInitParam::kLKneCh,
        56015.13300000001,
        1.5*0.0088964432, // 0.001 in psi,
        387.096,      // unit: mm^2 (0.6 in2)
        354.838       // unit: mm^2 (0.55 in2)
    };

    const CylinderParam kLAnk = {

        MpcInitParam::kLKneCl,
        MpcInitParam::kLKneCh,
        MpcInitParam::kLKneCl, // TODO: update it with balance param
        MpcInitParam::kLKneCh,
        57495.89467005076,
        0.0088964432, // 0.001 in psi,
        387.096,      // unit: mm^2 (0.6 in2)
        354.838       // unit: mm^2 (0.55 in2)
    };
    const ReservoirParam kLTank = {
        MpcInitParam::kLTankCl,
        MpcInitParam::kLTankCh
    };


    const CylinderParam kRKne = {

        MpcInitParam::kLKneCl,
        MpcInitParam::kLKneCh,
        MpcInitParam::kLKneCl, // TODO: update it with balance param
        MpcInitParam::kLKneCh,
        57619,
        0.0088964432, // 0.001 in psi,
        387.096,      // unit: mm^2 (0.6 in2)
        354.838       // unit: mm^2 (0.55 in2)
    };

    const CylinderParam kRAnk = {

        MpcInitParam::kLKneCl,
        MpcInitParam::kLKneCh,
        MpcInitParam::kLKneCl, // TODO: update it with balance param
        MpcInitParam::kLKneCh,
        57619,
        0.0088964432, // 0.001 in psi,
        387.096,      // unit: mm^2 (0.6 in2)
        354.838       // unit: mm^2 (0.55 in2)
    };
    const ReservoirParam kRTank = {
        MpcInitParam::kLTankCl,
        MpcInitParam::kLTankCh
    };

}
#endif