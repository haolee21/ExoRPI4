#ifndef MPC_PARAM_HPP
#define MPC_PARAM_HPP
#define MPC_STATE_NUM 14
#include <array>
namespace MpcInitParam
{
    const std::array<std::array<float,MPC_STATE_NUM>,2> kLTankCl
    {
        std::array<float,MPC_STATE_NUM>{-3.68753296e+00,  2.04860916e+03,  0.0,  6.87133562e+01, 0.0,  2.96822361e+02,  0.0, 0.0, -1.67566749e+01, 0.0, 0.0, 0.0, 0.0, -1.84158222e+02},
        std::array<float,MPC_STATE_NUM>{6.88073539e+01, -7.09547841e+02, -5.18057512e+02, 0.0, -4.46159693e+01,  0.0, 0.0, -4.12415795e+02, -9.71109568e-01, 0.0, 0.0, 5.13753700e+02, 0.0, -6.96589588e+01}
    };
    const std::array<std::array<float,MPC_STATE_NUM>,2> kLTankCh
    {
        std::array<float,MPC_STATE_NUM>{ 28.99804268, 0., 0., -87.42572126,0., -1260.80680865,0., -207.697389, 55.44576202, -64.66717725, 0., 0., 219.12535057, 0. },
        std::array<float,MPC_STATE_NUM>{ -2.9599239,  -2100.65091719,   -28.04066336,  0., -2.18224561, 201.13199447, 0., 1360.24025044, -24.59463977, 92.03466298, 0.,  0., -290.71015834, 0.}
    };

    const std::array<std::array<float,MPC_STATE_NUM>,2> kLKneCl
    {
        std::array<float,MPC_STATE_NUM>{8.44259165,0.,0.,144.18961872,0.,0.,0.,-1.74119718,-37.79086226,41.17903902,0.,278.22178418,0.,-54.11860612},
        std::array<float,MPC_STATE_NUM>{0.,0.,0.,0.,-243.88657005,0.,0.,0.,40.56873953,0.,-32.20674441,0.,-229.81397536,0.}

    };
    const std::array<std::array<float,MPC_STATE_NUM>,2> kLKneCh
    {
        std::array<float,MPC_STATE_NUM>{5.07536840e+00, 0.0,0.0, 0.0,-1.08115825e-01,-2.98224498e+01,-2.73864363e+02,0.0,0.0, 0.0,-1.53868307e+01, 0.0, 0.0, -1.62542842e+02},
        std::array<float,MPC_STATE_NUM>{-2.60843222e+00,  3.85343261e+02,  2.02098133e+01, 0.0, 9.21238925e-01, 3.50873063e+02,0.0, 0.0,0.0, 0.0, 0.0, 5.00963196e+02,0.0, 3.55336823e+01}

    };
    
}
#endif