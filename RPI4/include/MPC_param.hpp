#ifndef MPC_PARAM_HPP
#define MPC_PARAM_HPP

#include <array>
#include "ExoConfig.hpp"
namespace MpcInitParam
{
    //cl parameters are for model that decrease the pressure 
    //ch parameters are for model that increase the pressure
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLTankCh
    {
        std::array<double,MPC_STATE_NUM>{0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.04836662e-05,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.57126978e-04, -4.17982716e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -4.33944099e-05, -1.24781607e-04,  0.00000000e+00,
        6.14399807e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  5.42511379e-04,  0.00000000e+00,  0.00000000e+00,
       -6.03567527e-05,  0.00000000e+00,  1.02795589e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.28366122e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        8.74437281e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.69829752e-05,  7.08478027e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -8.80638348e-04,  1.34885641e-04,  0.00000000e+00,
        0.00000000e+00,  1.27598581e-04, -1.92023857e-03,  3.22927021e-04,
        0.00000000e+00,  0.00000000e+00,  2.32504893e-04, -1.66723124e-03,
        2.69000967e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -5.78225620e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  6.58587672e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.43528551e-03,
        0.00000000e+00,  4.67811541e-04,  0.00000000e+00,  6.70165623e-05,
        0.00000000e+00,  0.00000000e+00,  7.60980488e-03, -4.48747895e-03,
        2.78152099e-03,  0.00000000e+00,  0.00000000e+00,  9.07893621e-03,
       -9.42079399e-03,  1.27447072e-03,  0.00000000e+00,  0.00000000e+00,
        3.66480285e-03, -1.30822279e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  1.04262297e-03, -2.33222109e-04, -2.28154038e-03,
       -4.40228254e-04,  0.00000000e+00,  0.00000000e+00,  3.03240886e-04,
       -7.50627398e-03, -4.00286373e-04,  0.00000000e+00,  0.00000000e+00,
        2.19647220e-03, -1.38603866e-02,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  8.20187384e-03, -1.94923516e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.50973932e-02, -1.60958649e-02,
        4.10635009e-04,  0.00000000e+00,  0.00000000e+00,  1.03089792e-02,
       -7.67610378e-03, -2.84016096e-05,  0.00000000e+00,  0.00000000e+00,
        2.20289068e-03},
        std::array<double,MPC_STATE_NUM>{-3.13086142e-04,  6.40818056e-05,  0.00000000e+00,  0.00000000e+00,
        5.82751031e-05, -2.95426181e-04,  7.36506189e-06,  0.00000000e+00,
        0.00000000e+00,  4.65970929e-04, -3.26768021e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  4.04037369e-04, -2.55220775e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  7.35011961e-05,
       -2.59059414e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.35353706e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.56157733e-05,  3.30234879e-05,
        0.00000000e+00,  0.00000000e+00,  8.76284491e-07,  0.00000000e+00,
        4.23115709e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  1.75331347e-04,  0.00000000e+00,  0.00000000e+00,
        8.28576846e-06,  0.00000000e+00,  4.49749389e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -4.79530738e-05,  1.97530894e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -2.91302817e-04,
        8.73400813e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.75696364e-03,  1.25538266e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.26268675e-03,  4.03059734e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -8.88431612e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -7.11842735e-05,  0.00000000e+00,
        2.48850882e-05,  0.00000000e+00,  0.00000000e+00, -3.41197595e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  8.12549756e-04,
        0.00000000e+00,  2.01772490e-03,  0.00000000e+00,  0.00000000e+00,
        1.58195922e-04,  0.00000000e+00,  5.80102282e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -2.67214905e-03,  8.06247109e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -5.02555389e-03,
        1.31320908e-02, -5.35860217e-03,  0.00000000e+00,  0.00000000e+00,
       -5.63160684e-03,  1.55256503e-02, -1.05737916e-02,  0.00000000e+00,
        0.00000000e+00, -3.58828737e-03,  1.32828020e-02, -9.88969718e-03,
        0.00000000e+00,  0.00000000e+00, -2.70583681e-03,  6.54204648e-03,
       -3.37779988e-03,  0.00000000e+00,  0.00000000e+00, -2.16492830e-03,
        0.00000000e+00, -4.98764783e-06,  0.00000000e+00,  0.00000000e+00,
       -9.27259204e-04}
    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLTankCl
    {
        std::array<double,MPC_STATE_NUM>{-1.49759646e-03,  2.53008104e-03,  0.00000000e+00,  0.00000000e+00,
       -1.18237444e-03,  0.00000000e+00,  9.22771566e-04,  0.00000000e+00,
        0.00000000e+00, -8.76500075e-05,  0.00000000e+00,  9.40549272e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        4.13299192e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -5.84025028e-05,  2.98724970e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -2.00553075e-04,  1.55160960e-05,  0.00000000e+00,
        1.53624069e-05,  0.00000000e+00, -1.92452176e-05,  0.00000000e+00,
        0.00000000e+00,  5.41650025e-05, -4.16316161e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.21571430e-05, -6.83105191e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  5.02010969e-04,
       -3.36303702e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.41405384e-03, -1.65169196e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  2.94305432e-03, -7.50512859e-04,  2.32437594e-03,
        0.00000000e+00, -1.64227322e-03,  0.00000000e+00,  0.00000000e+00,
        1.96525667e-03,  0.00000000e+00, -6.77634900e-03,  0.00000000e+00,
        0.00000000e+00,  1.04135128e-03, -2.22407547e-04, -1.20327561e-02,
        0.00000000e+00,  0.00000000e+00,  2.13060743e-04, -1.06920231e-03,
       -2.98221945e-02,  0.00000000e+00,  4.24903453e-04,  0.00000000e+00,
       -8.46634865e-03, -1.02827405e-02,  0.00000000e+00,  1.36517487e-03,
        0.00000000e+00, -1.09852730e-02,  0.00000000e+00,  0.00000000e+00,
        4.42276230e-04,  0.00000000e+00, -1.01603009e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -3.08661444e-03,
        0.00000000e+00,  0.00000000e+00, -7.37499295e-04,  1.05823924e-03,
        0.00000000e+00,  0.00000000e+00, -1.00618784e-04, -2.32019757e-04,
        8.64374378e-03,  1.27298888e-04,  0.00000000e+00,  0.00000000e+00,
       -1.37986800e-03,  2.93069708e-03,  1.08690294e-02,  6.68009448e-03,
        0.00000000e+00, -4.21485463e-03,  0.00000000e+00,  1.83247567e-02,
        1.51603846e-02,  0.00000000e+00, -5.90241786e-03,  0.00000000e+00,
        1.48579741e-02,  8.17936408e-03,  0.00000000e+00, -9.01213585e-04,
        0.00000000e+00,  1.23546585e-02,  1.42202408e-03,  0.00000000e+00,
        0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{ -3.33423262e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        9.23117434e-04, -5.03043914e-05,  0.00000000e+00,  0.00000000e+00,
       -2.08072675e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -6.78628317e-06,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.90297123e-05,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        4.56936682e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  1.00234401e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  8.78047797e-05, -1.54921092e-05,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -5.30819473e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        7.30704525e-06, -1.67322835e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -3.31968861e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -2.88315751e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.62559700e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -8.37019206e-05, -1.33140010e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -5.69721552e-05, -3.21828763e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -7.11444578e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.41640533e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -2.51414944e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -3.48514516e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  3.75769000e-04, -6.26757616e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  3.21042444e-03,
       -7.14522416e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        4.39228400e-03, -7.78580151e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  5.28178706e-03, -5.72398424e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  3.54731606e-03, -2.35681983e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.07474233e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        5.70036388e-05}
    };

    const std::array<std::array<double,MPC_STATE_NUM>,2> kLKneCh
    {
        std::array<double,MPC_STATE_NUM>{ 8.01805166e-04,  0.00000000e+00,  0.00000000e+00,  1.46573557e-03,
       -1.17600019e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.44744507e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -3.38907406e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -8.02079330e-04,
        5.99733125e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  3.72509446e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.22014193e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -6.65845344e-05,
       -4.95386144e-03,  0.00000000e+00,  0.00000000e+00, -3.85930521e-05,
       -2.09057080e-03, -4.30492675e-03,  0.00000000e+00,  0.00000000e+00,
       -5.66774031e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.04731038e-02,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.10127383e-02,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -3.67614891e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.67372039e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -1.07216776e-02,
        0.00000000e+00,  0.00000000e+00, -2.11986164e-03, -4.95469664e-03,
        1.16073533e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  2.59738351e-02,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.28316285e-02,  0.00000000e+00,
        0.00000000e+00,  2.57847905e-04,  0.00000000e+00, -7.01680541e-04,
        0.00000000e+00,  0.00000000e+00,  4.04646826e-04,  0.00000000e+00,
       -1.96036485e-02,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        4.10050586e-06, -2.96699082e-02,  0.00000000e+00,  0.00000000e+00,
       -8.75066354e-04,  1.24686107e-02, -2.25828644e-02,  0.00000000e+00,
        0.00000000e+00, -1.66301377e-03,  1.76562948e-02, -1.24904101e-02,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.34434094e-02,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.35918784e-04},
        std::array<double,MPC_STATE_NUM>{-1.98904975e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  1.91079683e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  5.21568009e-04,  6.45161176e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.05825420e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -3.96187374e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -3.37629850e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.86210332e-05,
        0.00000000e+00,  0.00000000e+00,  7.72972014e-04,  0.00000000e+00,
        6.60489684e-04,  0.00000000e+00,  0.00000000e+00,  7.26853134e-03,
        0.00000000e+00,  1.19929041e-04,  0.00000000e+00,  0.00000000e+00,
        1.14590904e-03,  0.00000000e+00, -1.41381215e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -7.24709784e-04, -9.69127994e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.35501027e-03,
       -1.29823701e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -6.61449391e-03,  3.42595599e-03,  0.00000000e+00,  0.00000000e+00,
        1.31544494e-03,  0.00000000e+00,  1.91148288e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  6.86578422e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.41058587e-03,
       -9.43847490e-04,  0.00000000e+00,  0.00000000e+00, -1.01670653e-02,
        0.00000000e+00, -4.61289291e-03,  0.00000000e+00,  0.00000000e+00,
       -2.58536954e-02,  0.00000000e+00, -2.59966415e-03, -1.00101249e-03,
        0.00000000e+00, -1.25724438e-02,  0.00000000e+00,  7.30238363e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.66083487e-02,  0.00000000e+00,  0.00000000e+00,  1.78326295e-05,
        0.00000000e+00,  1.62829126e-02,  1.25648283e-02,  0.00000000e+00,
        7.98029593e-03, -1.50729064e-02,  1.62474932e-02,  8.33479197e-03,
        0.00000000e+00,  3.86418466e-04, -1.63239573e-02,  9.30932716e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -7.30144633e-03,
        5.82112785e-06,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -2.45118183e-04}

    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kLKneCl
    {
        std::array<double,MPC_STATE_NUM>{0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  4.23783810e-05,  0.00000000e+00,  0.00000000e+00,
        2.24289547e-05,  0.00000000e+00,  1.55935211e-03,  0.00000000e+00,
        0.00000000e+00,  3.35850998e-04,  0.00000000e+00,  1.05757012e-04,
        0.00000000e+00,  0.00000000e+00,  2.11204235e-05,  0.00000000e+00,
       -7.76098155e-05,  0.00000000e+00,  0.00000000e+00, -6.17971297e-06,
        0.00000000e+00, -3.30790653e-03,  0.00000000e+00,  0.00000000e+00,
       -2.09732434e-05, -1.27461222e-03, -6.85132235e-04,  0.00000000e+00,
        0.00000000e+00, -4.39939519e-05, -2.68617069e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        7.36735894e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.32054957e-04,  4.50849814e-03,  0.00000000e+00,  0.00000000e+00,
        5.73370767e-05,  3.56072277e-04, -2.13643975e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.29621973e-04, -1.42325497e-02,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -7.71601176e-03, -1.45930992e-03,  0.00000000e+00,  0.00000000e+00,
       -4.39201294e-03,  4.96932023e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.29711126e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  7.38362417e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.31132786e-02,
       -5.19912097e-03,  0.00000000e+00,  0.00000000e+00, -2.62635083e-03,
        0.00000000e+00, -2.01405265e-02, -2.00066353e-02,  0.00000000e+00,
       -4.29655730e-04,  0.00000000e+00, -6.82513686e-04, -2.87655354e-02,
        0.00000000e+00, -5.50720754e-04, -3.98330610e-03,  1.02101905e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -1.85247320e-06,
        2.72148410e-02,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.48444589e-03,  1.05836038e-02,  2.28632374e-02,  0.00000000e+00,
        0.00000000e+00, -4.79407986e-03,  0.00000000e+00,  2.00180142e-02,
        0.00000000e+00,  0.00000000e+00, -3.25874620e-03,  5.65248057e-04,
        6.42431429e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        4.30937106e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{1.45027039e-05,  0.00000000e+00,  0.00000000e+00,  6.00932193e-07,
        0.00000000e+00, -4.74784119e-06,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.35209082e-05,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.58267490e-06,  0.00000000e+00,
        1.29061286e-05,  0.00000000e+00,  0.00000000e+00,  1.80235626e-05,
        0.00000000e+00,  8.64205515e-06,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.00714408e-05,
        0.00000000e+00,  0.00000000e+00, -3.31046361e-05,  0.00000000e+00,
       -5.16436988e-05,  0.00000000e+00,  0.00000000e+00, -1.27091585e-04,
       -4.20342975e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  7.00382721e-04,  0.00000000e+00,
        0.00000000e+00,  2.93235030e-06,  0.00000000e+00,  1.41850817e-03,
        0.00000000e+00,  0.00000000e+00,  1.41631702e-05,  6.48511337e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -2.14992167e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -5.68297518e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -2.63511570e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.26951420e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  7.06879951e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  3.91242712e-04, -6.53278573e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.16667385e-02,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.07891337e-02,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  4.47423799e-04, -8.97969374e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  3.76837485e-03, -2.59769035e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  4.31077476e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.59151313e-03}

    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kRKneLAnkCh
    {
        std::array<double,MPC_STATE_NUM>{0.00000000e+00,  1.91748026e-04,  0.00000000e+00,  5.30088907e-04,
        0.00000000e+00, -4.91518960e-04,  4.22599517e-05,  0.00000000e+00,
        3.90245181e-07,  0.00000000e+00, -2.37887352e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.23129454e-03, -1.11055620e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  7.51217104e-04,
       -1.18490876e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.30433287e-05,  0.00000000e+00,  4.70712950e-04,  0.00000000e+00,
        8.41632519e-05,  0.00000000e+00,  0.00000000e+00,  4.31086297e-04,
        0.00000000e+00,  2.37557646e-04,  0.00000000e+00,  0.00000000e+00,
        4.70946799e-04,  0.00000000e+00,  6.10154688e-04,  0.00000000e+00,
        0.00000000e+00,  2.13291614e-04,  0.00000000e+00,  1.40791963e-03,
       -1.43836455e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.07056033e-04, -3.54599606e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.03612062e-03, -3.15325431e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -8.75084206e-04,
       -1.51865836e-04, -5.64090842e-04,  0.00000000e+00,  0.00000000e+00,
       -3.29563739e-04,  6.81160339e-06, -4.76622975e-05,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  6.74680377e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.35280749e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        4.33474379e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.36200694e-03,  3.92076589e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -1.09054191e-03,  6.17570481e-04,  4.85303543e-04,
        0.00000000e+00,  3.96457445e-05, -3.12653917e-04, -6.36296964e-04,
        1.34796747e-04,  0.00000000e+00,  0.00000000e+00, -7.74154125e-05,
       -4.78729199e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        7.10979533e-04, -1.58483367e-02,  2.31450973e-03,  0.00000000e+00,
        0.00000000e+00,  1.03204864e-02, -1.60588205e-02,  4.21110934e-03,
        0.00000000e+00,  0.00000000e+00,  8.52369230e-03, -1.37863605e-02,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00568830e-02,
       -4.96107713e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{ 0.00000000e+00,  2.92089123e-04,  0.00000000e+00,  9.34566445e-04,
       -2.16955442e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        7.00912822e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  2.51087914e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.48010946e-05,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -1.44244370e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -8.50364934e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -8.34976334e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -1.07373808e-07,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  2.72580037e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.52967256e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  1.85004848e-03,  0.00000000e+00,  8.73228294e-04,
        0.00000000e+00,  0.00000000e+00,  5.46341203e-05,  2.97727231e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.33537062e-04,  0.00000000e+00, -1.87393401e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.44900795e-03,
        0.00000000e+00,  0.00000000e+00,  7.31605662e-05,  0.00000000e+00,
       -5.80136779e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -5.02596537e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.94836629e-03,  0.00000000e+00,
        0.00000000e+00, -4.91803030e-04,  4.99020628e-04,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.57722407e-05,  7.81028297e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -4.11922799e-03,
        1.41785266e-02,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -8.67518985e-03,  8.25135452e-03,  7.98834504e-03,  0.00000000e+00,
        0.00000000e+00, -7.20865731e-03,  3.26406758e-03,  1.21979341e-02,
        0.00000000e+00,  0.00000000e+00, -6.07150995e-03,  5.88150196e-04,
        8.88086936e-03,  0.00000000e+00,  0.00000000e+00, -2.08426104e-04,
        1.82747807e-03,  1.93762589e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00}

    };
    const std::array<std::array<double,MPC_STATE_NUM>,2> kRKneLAnkCl
    {
        std::array<double,MPC_STATE_NUM>{-1.47165719e-04, -3.19964441e-05,  0.00000000e+00,  0.00000000e+00,
       -1.67409805e-04, -2.41051305e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -2.05794232e-05,
       -7.41712193e-06,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -8.04461127e-04, -1.35603885e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -5.53183175e-05, -1.03977062e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.40686506e-04,  0.00000000e+00,
        0.00000000e+00,  2.61415993e-04,  1.58818252e-04,  1.72212451e-03,
        0.00000000e+00,  0.00000000e+00,  1.79710891e-03,  1.75025718e-04,
       -1.73625747e-04, -8.31283894e-06,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00, -9.86257058e-04, -8.54328085e-03,  0.00000000e+00,
        0.00000000e+00, -1.85133006e-03,  0.00000000e+00, -4.06766401e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  4.39213635e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  3.41752904e-04,
        7.71758560e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        9.55363869e-03,  0.00000000e+00, -3.21196464e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -3.02614104e-02,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
       -7.28953897e-03,  0.00000000e+00,  0.00000000e+00, -1.33311458e-02,
        2.55549239e-02,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  7.98009969e-03,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  3.24042540e-02,  0.00000000e+00, -1.79982012e-02,
        0.00000000e+00, -3.78801736e-02,  3.34064727e-02,  0.00000000e+00,
       -5.66491002e-02,  0.00000000e+00, -2.35513814e-02,  8.94928145e-03,
       -5.96390870e-04, -1.80834161e-03,  0.00000000e+00, -6.55270852e-03,
       -5.90413526e-03,  1.73063405e-02,  3.70099342e-02,  0.00000000e+00,
        0.00000000e+00, -5.39044129e-03,  5.38634685e-03,  9.00436373e-02,
        0.00000000e+00,  0.00000000e+00, -5.07850934e-02,  5.32284972e-04,
        4.58339708e-02,  0.00000000e+00,  0.00000000e+00, -2.95047767e-02,
        3.01161158e-03,  2.84265110e-05,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00},
        std::array<double,MPC_STATE_NUM>{ 1.49822895e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  3.82671112e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  1.14077839e-05,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.73668822e-06,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.70852524e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  1.67383654e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  8.54560602e-06,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.71284150e-05,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        1.48932105e-04,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  2.34768406e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00, -1.07117775e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -3.52279972e-04,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  2.68544899e-04,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  2.59251148e-03, -4.19285622e-05,
        0.00000000e+00,  0.00000000e+00, -7.18596988e-06,  0.00000000e+00,
       -5.65030546e-03,  0.00000000e+00,  0.00000000e+00, -2.08935756e-04,
        0.00000000e+00, -1.30574290e-03, -7.79407185e-05,  0.00000000e+00,
       -1.73051661e-03,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  3.81910530e-03,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  6.46969586e-03,
       -7.68885864e-03,  0.00000000e+00,  1.27613063e-03,  0.00000000e+00,
        1.81629997e-03, -2.41409242e-02,  0.00000000e+00,  1.15293679e-03,
        0.00000000e+00,  1.43565349e-02, -1.35147143e-02,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  4.48139786e-03, -3.91248681e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  6.18577314e-05,
       -6.58432444e-05,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        2.87150261e-05}

    };
    
    const double kLkneMaxLen = 57132.776000000005;
    
}
#endif