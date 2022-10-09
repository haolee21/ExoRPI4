#ifndef MPC_PARAM_HPP
#define MPC_PARAM_HPP
#define MPC_STATE_NUM 125
#define MPC_DELAY 25
#include <array>
namespace MpcInitParam
{
    //cl parameters are for model that decrease the pressure 
    //ch parameters are for model that increase the pressure
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLTankCh
    {
        std::array<double,MPC_STATE_NUM>{-1.16311528e-05, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -3.07344889e-04,  8.28115675e-05, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  4.25393714e-04, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  1.40987089e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        1.79530879e-04, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -3.19950513e-04, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -2.89694129e-05, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -3.75713225e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -5.24026652e-04,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -5.13856773e-05,  9.18546942e-04, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  2.01904319e-03, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -2.11860042e-03,  7.85487898e-04,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -5.24256382e-04, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -2.37516320e-03, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -1.62919053e-03, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  1.13054731e-03,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        1.92634546e-03, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        2.41583191e-03,  5.95175260e-03, -0.00000000e+00, -0.00000000e+00,
       -2.31836594e-02,  0.00000000e+00,  2.33343627e-03, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  3.60177229e-03,
       -5.88816832e-03,  0.00000000e+00,  0.00000000e+00,  1.73113725e-02,
        0.00000000e+00, -4.68441991e-03,  0.00000000e+00,  0.00000000e+00,
        4.17960181e-03, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        1.43468489e-02,  1.44834212e-02, -1.76522373e-02, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.08118818e-02, -9.05919399e-03,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  7.10592878e-03,
       -0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{-4.10185389e-05,  0.00000000e+00,  0.00000000e+00,  1.04997092e-03,
       -5.18959654e-05, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -5.22014549e-04, -2.19211749e-04,  0.00000000e+00,
        0.00000000e+00,  9.97395547e-04, -0.00000000e+00, -6.10066446e-05,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -3.24502809e-05,
       -1.71435731e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -6.90585761e-05, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -2.32293435e-04,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -5.87217542e-05, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -1.94961395e-04, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -8.62247676e-06, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  1.41174361e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        5.13091547e-05, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -9.59966202e-05, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -4.80631177e-04, -0.00000000e+00,
       -0.00000000e+00, -1.39441949e-03, -0.00000000e+00, -2.50803432e-04,
       -0.00000000e+00, -0.00000000e+00, -3.90409094e-03, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -4.00100811e-03,
       -0.00000000e+00,  2.03764739e-04, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  4.82317462e-04, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  1.07972683e-03,  0.00000000e+00,
       -0.00000000e+00, -1.95698016e-02, -6.75448825e-03,  9.83835649e-03,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        5.72575661e-03, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  3.78355872e-03, -0.00000000e+00, -0.00000000e+00,
       -1.40834427e-02, -0.00000000e+00,  2.91594515e-03,  1.89060677e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        5.45699455e-04, -0.00000000e+00, -0.00000000e+00, -5.78009296e-03,
       -0.00000000e+00}
    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLTankCl
    {
        std::array<double,MPC_STATE_NUM>{1.18231122e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  1.50351632e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -1.35941682e-04, -6.41390164e-05,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -5.87769048e-04,
       -4.79299345e-05,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  6.10770446e-04,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -1.79051632e-03,  6.13383943e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -7.19124060e-04,
        4.62127405e-04,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -3.64662968e-04, -1.82512823e-04,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -1.06546958e-03, -5.06595658e-04,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -9.66379807e-04,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  8.58667060e-04,  0.00000000e+00, -0.00000000e+00,
       -1.13694644e-02,  2.85158502e-03,  1.15950129e-03,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  6.80347307e-04,
       -4.28350107e-03, -5.05194643e-03, -0.00000000e+00, -0.00000000e+00,
        1.07140324e-02, -4.82929439e-03, -0.00000000e+00, -0.00000000e+00,
       -8.19192250e-03,  8.18436998e-03, -3.47363386e-03, -0.00000000e+00,
       -0.00000000e+00, -8.38478203e-04,  5.01712368e-03,  2.80025494e-06,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        3.30606984e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -3.56938608e-03,  8.06856224e-03,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -1.75992457e-02,  0.00000000e+00,  4.70324284e-02,
        0.00000000e+00, -0.00000000e+00, -1.61124705e-03, -0.00000000e+00,
        1.90322128e-02,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -9.57324941e-04,  7.77789053e-03,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{ 0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        1.44358061e-06, -8.48484344e-06, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -2.15457866e-06, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.17905647e-05,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -2.53961514e-05, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.49999576e-04, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -3.09293569e-05, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -2.66701731e-05, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  1.32320169e-04, -3.71770380e-05,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -3.04317826e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        7.04085763e-04, -7.62128612e-05, -2.70588170e-03, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -2.06112835e-03,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  4.06305233e-04,
       -5.95316414e-03, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        4.48099824e-04, -9.69490420e-03, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  4.02607572e-05, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -5.80999109e-03,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -1.31238473e-03,
       -1.21813326e-03,  0.00000000e+00,  7.03613279e-03, -0.00000000e+00,
       -4.87400273e-04, -1.76767465e-03,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -1.23404475e-02,  0.00000000e+00,
        3.47572588e-03, -0.00000000e+00,  0.00000000e+00, -3.28859283e-03,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00,  1.48138181e-03,
       -1.06873045e-02,  0.00000000e+00,  0.00000000e+00, -3.18395134e-03,
        2.73708883e-05, -3.52664421e-03,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00}
    };

    const std::array<std::array<double,MPC_STATE_NUM>,2> kLKneCh
    {
        std::array<double,MPC_STATE_NUM>{2.66367900e-04,  0.00000000e+00, -0.00000000e+00, -4.15906285e-03,
        5.37245028e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  3.93588764e-04,  7.21058676e-06, -0.00000000e+00,
       -0.00000000e+00, -7.10689133e-03,  3.19423485e-03,  2.07078968e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -5.85162869e-03,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -1.98785255e-03,  1.71668422e-03, -3.35878396e-04, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  2.62225242e-04,  7.36553333e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -2.88789158e-04, -0.00000000e+00, -0.00000000e+00,
       -9.96106403e-05, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -3.40078217e-04, -7.78853010e-04,
       -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        1.12951849e-03, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  1.83852465e-03, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  6.01555267e-04, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -1.89793592e-03,
       -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -2.89521946e-03, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  1.59531807e-03, -0.00000000e+00, -0.00000000e+00,
       -1.57336581e-03, -0.00000000e+00,  3.08880331e-03,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  1.51146415e-03,  2.85430802e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        9.25699228e-03, -7.34226059e-03, -0.00000000e+00,  0.00000000e+00,
        2.48905385e-02, -3.34413612e-04,  2.32011461e-03, -0.00000000e+00,
        3.38246364e-02,  0.00000000e+00, -2.41112180e-02, -5.45033711e-04,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -9.73726670e-03,
       -4.67522141e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.72522960e-03},
        std::array<double,MPC_STATE_NUM>{-3.12658559e-04,  0.00000000e+00,  0.00000000e+00,  1.94387101e-03,
       -6.26121312e-04, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -3.51159569e-04, -1.73812970e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.79284764e-03, -0.00000000e+00,
        1.70219759e-04,  0.00000000e+00,  0.00000000e+00,  1.43760608e-03,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -1.11909702e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -3.76621614e-04,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        5.70690397e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  7.08515570e-04,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -7.68850642e-05,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -1.26736140e-03,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -1.27778850e-03,  0.00000000e+00, -0.00000000e+00,
       -2.16259300e-03,  0.00000000e+00,  2.14931964e-04,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  9.43866809e-04,  4.91350447e-04,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  5.19070981e-03,
       -5.82821163e-06,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -1.98454336e-03,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -3.88728356e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -2.64411421e-03,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        4.23715037e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  7.34605336e-03,  0.00000000e+00, -0.00000000e+00,
       -2.17308386e-02,  0.00000000e+00,  9.24441215e-04,  5.93266675e-03,
       -0.00000000e+00, -1.02655359e-02,  1.59853815e-02,  1.58362662e-03,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  6.69387370e-03,
        5.86134349e-04,  0.00000000e+00, -0.00000000e+00, -5.90168432e-03,
       -0.00000000e+00}

    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLKneCl
    {
        std::array<double,MPC_STATE_NUM>{9.49889781e-05, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  3.98819607e-06,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -1.11657884e-06,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -1.04166008e-05,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        1.28586741e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  4.53635708e-04,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -3.38305725e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -3.35991464e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -1.27088613e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  7.63764960e-04,  0.00000000e+00,
        0.00000000e+00,  1.78682358e-03, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.59583479e-03,  0.00000000e+00,
       -6.51458727e-04, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.44668150e-03,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -2.43419949e-05,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.41275705e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  7.94303285e-04,
        4.33528475e-04,  0.00000000e+00,  0.00000000e+00,  1.19946803e-03,
        0.00000000e+00, -4.78981357e-04, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -5.47404105e-03, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  8.47441335e-03, -2.13239231e-03,
       -5.06940664e-03, -0.00000000e+00, -1.43817258e-02,  0.00000000e+00,
       -1.07808635e-03, -0.00000000e+00, -0.00000000e+00, -4.25815337e-03,
       -0.00000000e+00,  6.42578819e-03,  0.00000000e+00, -0.00000000e+00,
       -3.88004600e-03, -9.46887628e-03,  2.16848395e-03,  2.61909913e-02,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  7.48497587e-05,
        5.09267578e-02,  0.00000000e+00, -0.00000000e+00, -6.08558853e-04,
       -2.19626944e-05,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{-8.63426657e-05, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -7.47853072e-05,  3.63275892e-04, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  4.79546039e-05, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -5.07758889e-04,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -5.50312041e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -2.10167821e-04,  7.18565334e-05, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -2.19240691e-04,  1.82432021e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        1.71305730e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -2.86786306e-04, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -9.25481022e-05, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  1.34706347e-05,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.35057090e-04, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -5.51753982e-04,  5.26255477e-04, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -7.83147910e-05, -4.54969772e-04, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -8.98869060e-04,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -3.52607244e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  1.52270253e-04, -4.43988528e-04, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  5.58604590e-04, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -1.08567966e-03, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -1.77248706e-02,  0.00000000e+00,  0.00000000e+00,
        3.06183036e-03, -6.54271593e-04, -1.89044107e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -3.56574927e-02,
        0.00000000e+00,  1.07420172e-02,  0.00000000e+00,  6.78278193e-04,
       -1.10395518e-02,  0.00000000e+00,  0.00000000e+00, -2.71334807e-03,
        0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -3.75830881e-04}

    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLKneAnkCh
    {
        std::array<double,MPC_STATE_NUM>{3.45247866e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.90129997e-03, -4.75385926e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -6.96984901e-05,  1.54114712e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -6.49219447e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        3.62502935e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        7.25328770e-05,  1.54082890e-04, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -1.48744297e-04, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -3.53190034e-04,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -4.77235687e-04,
       -2.36246006e-04,  0.00000000e+00, -0.00000000e+00, -5.49911113e-03,
       -0.00000000e+00, -4.71474677e-04,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  7.02034899e-04,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -1.72262285e-04,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  3.93621661e-03,
        1.58221065e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -7.62248405e-04,  0.00000000e+00,  0.00000000e+00,
        2.27260512e-03, -0.00000000e+00, -1.24589645e-03,  0.00000000e+00,
        0.00000000e+00,  1.53945874e-03,  0.00000000e+00, -3.54055553e-03,
        2.47222711e-02,  0.00000000e+00,  0.00000000e+00,  4.19485135e-03,
        6.78287490e-05,  2.08403848e-02, -0.00000000e+00, -2.16410977e-02,
        0.00000000e+00,  4.33682620e-03,  0.00000000e+00, -0.00000000e+00,
       -5.67591988e-02, -0.00000000e+00,  0.00000000e+00,  6.39681424e-02,
       -0.00000000e+00, -0.00000000e+00, -4.28390565e-03, -6.29156811e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  4.40193210e-03,
       -6.04888387e-03,  2.48380455e-02,  0.00000000e+00,  4.57362528e-02,
        5.31642876e-03, -5.66592579e-03,  0.00000000e+00,  0.00000000e+00,
        5.04064128e-02,  5.53627102e-03,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -5.00473577e-03,  1.80787657e-02,
       -1.88847823e-01, -0.00000000e+00, -0.00000000e+00, -3.72064264e-02,
       -3.87286446e-03, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        1.80660036e-02},
        std::array<double,MPC_STATE_NUM>{2.06324743e-04,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -6.60350404e-04,  7.95067492e-05,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -1.90826938e-04, -2.27682877e-04,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  1.86281196e-04,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  2.20976000e-04,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  8.05938416e-05,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -1.46346727e-05,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -8.64341429e-04,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -4.38760742e-04,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  3.09959838e-04,  4.41261487e-05,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        9.43001529e-04,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        2.61090189e-04,  3.85917218e-04,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  5.34582417e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  7.69200204e-04, -1.30328292e-03,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00,  2.63720232e-03,
       -2.04672015e-03,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        2.18787140e-03, -1.11153392e-03,  0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -1.94638274e-04,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.83517975e-02,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        1.97721209e-03,  3.58382679e-02,  0.00000000e+00, -2.36968713e-02,
       -3.31420803e-03,  2.21828387e-03,  0.00000000e+00,  0.00000000e+00,
       -6.17909869e-03, -4.39285333e-04,  0.00000000e+00,  2.01732561e-02,
        0.00000000e+00, -3.77273658e-04, -2.81030020e-04, -0.00000000e+00,
        5.67843249e-03,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -5.33096075e-05,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00}

    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLKneAnkCl
    {
        std::array<double,MPC_STATE_NUM>{-3.18303196e-04, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00,  9.41433133e-04, -2.33671123e-03, -0.00000000e+00,
        0.00000000e+00, -1.66304211e-03,  2.95013553e-04, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -4.88889536e-04,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        3.08320885e-04, -1.44968199e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.63960192e-03,  3.82491922e-04,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        6.50300915e-04,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00,  4.29428915e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.91886303e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  5.69071119e-03, -2.17085998e-03,
        1.34341780e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -4.69927247e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  2.06069152e-03,  0.00000000e+00, -0.00000000e+00,
       -8.17349369e-03, -0.00000000e+00,  1.88499466e-03,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  3.46271039e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -2.52467278e-03,  0.00000000e+00, -0.00000000e+00,  0.00000000e+00,
        4.45481149e-03, -6.84566641e-03,  0.00000000e+00, -0.00000000e+00,
        1.71830428e-02,  0.00000000e+00, -7.01354005e-03,  0.00000000e+00,
       -0.00000000e+00,  8.50787855e-03,  0.00000000e+00,  1.24195727e-03,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -2.74633270e-03,
        6.87578973e-03,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -6.33295283e-03,  2.65558252e-02,  0.00000000e+00, -0.00000000e+00,
       -2.74418095e-02, -5.52858498e-02, -0.00000000e+00,  2.25342788e-02,
       -0.00000000e+00, -0.00000000e+00,  3.68813776e-03, -2.57969891e-02,
        1.23796603e-02, -0.00000000e+00, -2.23657163e-02,  8.51884179e-02,
        7.56254224e-03,  1.06190884e-02,  0.00000000e+00,  0.00000000e+00,
       -2.71176379e-02},
        std::array<double,MPC_STATE_NUM>{-1.47346193e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        1.91392094e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  3.98675906e-04,
       -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        3.44689033e-04, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -7.64823323e-04,  0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
       -0.00000000e+00, -5.22113273e-05, -2.87293534e-04, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -2.13738092e-04,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  5.00213918e-04, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.90994143e-04, -0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00,  1.60916558e-04,
       -0.00000000e+00, -0.00000000e+00,  0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -8.07317354e-03, -0.00000000e+00,  0.00000000e+00,
       -0.00000000e+00, -6.97685594e-04, -0.00000000e+00, -0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -3.86873771e-04, -0.00000000e+00,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.81367955e-04,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.04564958e-03, -0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  5.12513957e-04, -0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -2.65367091e-03,
        0.00000000e+00,  0.00000000e+00, -1.32071602e-03, -0.00000000e+00,
       -1.33732611e-02,  0.00000000e+00,  7.83193290e-06, -0.00000000e+00,
       -2.79188000e-04, -3.46030683e-02,  0.00000000e+00,  1.39753170e-02,
       -0.00000000e+00, -1.10166930e-03, -2.37070106e-02,  0.00000000e+00,
        1.09950179e-02, -0.00000000e+00, -0.00000000e+00, -1.50053356e-02,
        0.00000000e+00,  0.00000000e+00, -0.00000000e+00, -2.50559663e-04,
       -0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -1.02013288e-04,
        0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  6.90376378e-04,
       -0.00000000e+00}

    };
    
    const double kLkneMaxLen = 57815.36248175182;
    
}
#endif