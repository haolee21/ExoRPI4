#include <cmath>
#include <iostream>

#include "MPC.hpp"
#include "MPC_param.hpp"

using namespace std;

// const double MPC::kArea =  0.31*645.16f;  //unit: mm^2

MPC::MPC(std::array<std::array<double, MPC_STATE_NUM>, 2> cl, std::array<std::array<double, MPC_STATE_NUM>, 2> ch, std::string file_name)
    : ah(ch[0]), bh(ch[1]), al(cl[0]), bl(cl[1]),
      mpc_rec(file_name, MPC_HEAD)

{
    // this->max_pos = param.max_pos;
    // this->max_len_mm = this->GetLenLinear_mm(max_pos);

    // setup osqp solver
    this->osqp_data.reset(new OSQPData);
    this->osqp_settings.reset(new OSQPSettings);
    osqp_set_default_settings(this->osqp_settings.get());
    this->osqp_settings->alpha = 1.0;
    this->osqp_settings->verbose = false;

    this->phi_scale = 1.0;
    // this->pre_pos = 0.0;
    this->meas_idx = 0;
}

MPC::~MPC()
{
    // clean up osqp
    osqp_cleanup(work);
    if (this->osqp_data.get())
    {
        if (this->osqp_data->A)
            c_free(this->osqp_data->A);
        if (this->osqp_data->P)
            c_free(this->osqp_data->P);
        c_free(this->osqp_data.get());
    }
}
void MPC::UpdateParamH(array<double, MPC_STATE_NUM> new_a, array<double, MPC_STATE_NUM> new_b)
{

    this->ah = new_a;
    this->bh = new_b;
}
void MPC::UpdateParamL(array<double, MPC_STATE_NUM> new_a, array<double, MPC_STATE_NUM> new_b)
{
    this->al = new_a;
    this->bl = new_b;
}
Eigen::Matrix<double, 2, 1> MPC::UpdateF(const double *p_h, const double *p_l,
                                         const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    // Record the old Phi
    double x0 = p_h[0] * u[0];
    double x1 = x0 / p_l[0];
    double x2 = p_h[2] * u[2];
    double x3 = x2 / p_l[2];
    double x4 = p_h[3] * u[3];
    double x5 = x4 / p_l[3];
    double x6 = p_h[4] * u[4];
    double x7 = x6 / p_l[4];
    double x8 = p_h[5] * u[5];
    double x9 = x8 / p_l[5];
    double x10 = p_h[6] * u[6];
    double x11 = x10 / p_l[6];
    double x12 = p_h[7] * u[7];
    double x13 = x12 / p_l[7];
    double x14 = p_h[8] * u[8];
    double x15 = x14 / p_l[8];
    double x16 = p_h[9] * u[9];
    double x17 = x16 / p_l[9];
    double x18 = p_h[10] * u[10];
    double x19 = x18 / p_l[10];
    double x20 = p_h[11] * u[11];
    double x21 = x20 / p_l[11];
    double x22 = p_h[1] * u[1];
    double x23 = x22 / p_l[1];
    double x24 = p_h[12] * u[12];
    double x25 = x24 / p_l[12];
    double x26 = p_h[13] * u[13];
    double x27 = x26 / p_l[13];
    double x28 = p_h[14] * u[14];
    double x29 = x28 / p_l[14];
    double x30 = 1 - p_l[2] / p_h[2];
    double x31 = x2 * x30;
    double x32 = 1 - p_l[3] / p_h[3];
    double x33 = x32 * x4;
    double x34 = 1 - p_l[0] / p_h[0];
    double x35 = x0 * x34;
    double x36 = 1 - p_l[4] / p_h[4];
    double x37 = x36 * x6;
    double x38 = 1 - p_l[5] / p_h[5];
    double x39 = x38 * x8;
    double x40 = 1 - p_l[6] / p_h[6];
    double x41 = x10 * x40;
    double x42 = 1 - p_l[7] / p_h[7];
    double x43 = x12 * x42;
    double x44 = 1 - p_l[8] / p_h[8];
    double x45 = x14 * x44;
    double x46 = 1 - p_l[9] / p_h[9];
    double x47 = x16 * x46;
    double x48 = 1 - p_l[10] / p_h[10];
    double x49 = x18 * x48;
    double x50 = 1 - p_l[11] / p_h[11];
    double x51 = x20 * x50;
    double x52 = 1 - p_l[12] / p_h[12];
    double x53 = x24 * x52;
    double x54 = 1 - p_l[13] / p_h[13];
    double x55 = x26 * x54;
    double x56 = 1 - p_l[1] / p_h[1];
    double x57 = x22 * x56;
    double x58 = 1 - p_l[14] / p_h[14];
    double x59 = x28 * x58;
    double x60 = (p_h[2] * p_h[2]);
    double x61 = -(p_l[2] * p_l[2]) / x60 + 1;
    double x62 = u[2] * x61;
    double x63 = (p_h[3] * p_h[3]);
    double x64 = -(p_l[3] * p_l[3]) / x63 + 1;
    double x65 = u[3] * x64;
    double x66 = (p_h[4] * p_h[4]);
    double x67 = -(p_l[4] * p_l[4]) / x66 + 1;
    double x68 = u[4] * x67;
    double x69 = (p_h[5] * p_h[5]);
    double x70 = -(p_l[5] * p_l[5]) / x69 + 1;
    double x71 = u[5] * x70;
    double x72 = (p_h[6] * p_h[6]);
    double x73 = -(p_l[6] * p_l[6]) / x72 + 1;
    double x74 = u[6] * x73;
    double x75 = (p_h[7] * p_h[7]);
    double x76 = -(p_l[7] * p_l[7]) / x75 + 1;
    double x77 = u[7] * x76;
    double x78 = (p_h[8] * p_h[8]);
    double x79 = -(p_l[8] * p_l[8]) / x78 + 1;
    double x80 = u[8] * x79;
    double x81 = (p_h[9] * p_h[9]);
    double x82 = -(p_l[9] * p_l[9]) / x81 + 1;
    double x83 = u[9] * x82;
    double x84 = (p_h[0] * p_h[0]);
    double x85 = -(p_l[0] * p_l[0]) / x84 + 1;
    double x86 = u[0] * x85;
    double x87 = (p_h[10] * p_h[10]);
    double x88 = -(p_l[10] * p_l[10]) / x87 + 1;
    double x89 = u[10] * x88;
    double x90 = (p_h[11] * p_h[11]);
    double x91 = -(p_l[11] * p_l[11]) / x90 + 1;
    double x92 = u[11] * x91;
    double x93 = (p_h[12] * p_h[12]);
    double x94 = -(p_l[12] * p_l[12]) / x93 + 1;
    double x95 = u[12] * x94;
    double x96 = (p_h[13] * p_h[13]);
    double x97 = -(p_l[13] * p_l[13]) / x96 + 1;
    double x98 = u[13] * x97;
    double x99 = (p_h[14] * p_h[14]);
    double x100 = -(p_l[14] * p_l[14]) / x99 + 1;
    double x101 = u[14] * x100;
    double x102 = (p_h[1] * p_h[1]);
    double x103 = -(p_l[1] * p_l[1]) / x102 + 1;
    double x104 = u[1] * x103;
    double x105 = (u[2] * u[2]);
    double x106 = p_h[2] * x105 * x61;
    double x107 = (u[3] * u[3]);
    double x108 = p_h[3] * x107 * x64;
    double x109 = (u[4] * u[4]);
    double x110 = p_h[4] * x109 * x67;
    double x111 = (u[5] * u[5]);
    double x112 = p_h[5] * x111 * x70;
    double x113 = (u[6] * u[6]);
    double x114 = p_h[6] * x113 * x73;
    double x115 = (u[7] * u[7]);
    double x116 = p_h[7] * x115 * x76;
    double x117 = (u[0] * u[0]);
    double x118 = p_h[0] * x117 * x85;
    double x119 = (u[8] * u[8]);
    double x120 = p_h[8] * x119 * x79;
    double x121 = (u[9] * u[9]);
    double x122 = p_h[9] * x121 * x82;
    double x123 = (u[10] * u[10]);
    double x124 = p_h[10] * x123 * x88;
    double x125 = (u[11] * u[11]);
    double x126 = p_h[11] * x125 * x91;
    double x127 = (u[12] * u[12]);
    double x128 = p_h[12] * x127 * x94;
    double x129 = (u[13] * u[13]);
    double x130 = p_h[13] * x129 * x97;
    double x131 = (u[14] * u[14]);
    double x132 = p_h[14] * x100 * x131;
    double x133 = (u[1] * u[1]);
    double x134 = p_h[1] * x103 * x133;
    double x135 = x105 * (x30 * x30) * x60;
    double x136 = x107 * (x32 * x32) * x63;
    double x137 = x109 * (x36 * x36) * x66;
    double x138 = x111 * (x38 * x38) * x69;
    double x139 = x117 * (x34 * x34) * x84;
    double x140 = x113 * (x40 * x40) * x72;
    double x141 = x115 * (x42 * x42) * x75;
    double x142 = x119 * (x44 * x44) * x78;
    double x143 = x121 * (x46 * x46) * x81;
    double x144 = x123 * (x48 * x48) * x87;
    double x145 = x125 * (x50 * x50) * x90;
    double x146 = x127 * (x52 * x52) * x93;
    double x147 = x129 * (x54 * x54) * x96;
    double x148 = x131 * (x58 * x58) * x99;
    double x149 = x102 * x133 * (x56 * x56);
    Eigen::Matrix<double, 2, 1> f;
    f << a[0] * x1 + a[10] * x3 + a[11] * x31 + a[12] * x135 + a[13] * x106 + a[14] * x62 + a[15] * x5 + a[16] * x33 + a[17] * x136 + a[18] * x108 + a[19] * x65 + a[1] * x35 + a[20] * x7 + a[21] * x37 + a[22] * x137 + a[23] * x110 + a[24] * x68 + a[25] * x9 + a[26] * x39 + a[27] * x138 + a[28] * x112 + a[29] * x71 + a[2] * x139 + a[30] * x11 + a[31] * x41 + a[32] * x140 + a[33] * x114 + a[34] * x74 + a[35] * x13 + a[36] * x43 + a[37] * x141 + a[38] * x116 + a[39] * x77 + a[3] * x118 + a[40] * x15 + a[41] * x45 + a[42] * x142 + a[43] * x120 + a[44] * x80 + a[45] * x17 + a[46] * x47 + a[47] * x143 + a[48] * x122 + a[49] * x83 + a[4] * x86 + a[50] * x19 + a[51] * x49 + a[52] * x144 + a[53] * x124 + a[54] * x89 + a[55] * x21 + a[56] * x51 + a[57] * x145 + a[58] * x126 + a[59] * x92 + a[5] * x23 + a[60] * x25 + a[61] * x53 + a[62] * x146 + a[63] * x128 + a[64] * x95 + a[65] * x27 + a[66] * x55 + a[67] * x147 + a[68] * x130 + a[69] * x98 + a[6] * x57 + a[70] * x29 + a[71] * x59 + a[72] * x148 + a[73] * x132 + a[74] * x101 + a[7] * x149 + a[8] * x134 + a[9] * x104, b[0] * x1 + b[10] * x3 + b[11] * x31 + b[12] * x135 + b[13] * x106 + b[14] * x62 + b[15] * x5 + b[16] * x33 + b[17] * x136 + b[18] * x108 + b[19] * x65 + b[1] * x35 + b[20] * x7 + b[21] * x37 + b[22] * x137 + b[23] * x110 + b[24] * x68 + b[25] * x9 + b[26] * x39 + b[27] * x138 + b[28] * x112 + b[29] * x71 + b[2] * x139 + b[30] * x11 + b[31] * x41 + b[32] * x140 + b[33] * x114 + b[34] * x74 + b[35] * x13 + b[36] * x43 + b[37] * x141 + b[38] * x116 + b[39] * x77 + b[3] * x118 + b[40] * x15 + b[41] * x45 + b[42] * x142 + b[43] * x120 + b[44] * x80 + b[45] * x17 + b[46] * x47 + b[47] * x143 + b[48] * x122 + b[49] * x83 + b[4] * x86 + b[50] * x19 + b[51] * x49 + b[52] * x144 + b[53] * x124 + b[54] * x89 + b[55] * x21 + b[56] * x51 + b[57] * x145 + b[58] * x126 + b[59] * x92 + b[5] * x23 + b[60] * x25 + b[61] * x53 + b[62] * x146 + b[63] * x128 + b[64] * x95 + b[65] * x27 + b[66] * x55 + b[67] * x147 + b[68] * x130 + b[69] * x98 + b[6] * x57 + b[70] * x29 + b[71] * x59 + b[72] * x148 + b[73] * x132 + b[74] * x101 + b[7] * x149 + b[8] * x134 + b[9] * x104;
    return f;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[71] * u[14];
    double x1 = (p_l[14] * p_l[14]);
    double x2 = p_h[14] * u[14] / x1;
    double x3 = 2 * u[14];
    double x4 = a[74] * x3;
    double x5 = (p_h[14]);
    double x6 = p_l[14] * x5;
    double x7 = p_l[14] / p_h[14];
    double x8 = (u[14] * u[14]);
    double x9 = 2 * x8;
    double x10 = a[73] * x9;
    double x11 = 1 - x7;
    double x12 = p_h[14] * x9;
    double x13 = a[72] * x12;
    double x14 = u[14] / p_l[14];
    double x15 = x1 / (p_h[14] * p_h[14] * p_h[14]);
    double x16 = x1 * x5;
    double x17 = x8 * (1 - x16);
    double x18 = p_l[14] * x11 * x9;
    double x19 = (x11 * x11);
    double x20 = b[71] * u[14];
    double x21 = b[74] * x3;
    double x22 = b[73] * x9;
    double x23 = b[72] * x12;
    Eigen::Matrix<double, 2, 2> dF_dx_T;
    dF_dx_T << -a[70] * x2 - x0 - x10 * x7 - x11 * x13 - x4 * x6, a[70] * x14 + a[72] * x18 + a[73] * x17 + x0 * x11 + x0 * x7 + x10 * x16 + x13 * x19 + x15 * x4, -b[70] * x2 - x11 * x23 - x20 - x21 * x6 - x22 * x7, b[70] * x14 + b[72] * x18 + b[73] * x17 + x11 * x20 + x15 * x21 + x16 * x22 + x19 * x23 + x20 * x7;
    return dF_dx_T;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL1_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[66] * u[13];
    double x1 = (p_l[13] * p_l[13]);
    double x2 = p_h[13] * u[13] / x1;
    double x3 = 2 * u[13];
    double x4 = a[69] * x3;
    double x5 = (p_h[13]);
    double x6 = p_l[13] * x5;
    double x7 = p_l[13] / p_h[13];
    double x8 = (u[13] * u[13]);
    double x9 = 2 * x8;
    double x10 = a[68] * x9;
    double x11 = 1 - x7;
    double x12 = p_h[13] * x9;
    double x13 = a[67] * x12;
    double x14 = u[13] / p_l[13];
    double x15 = x1 / (p_h[13] * p_h[13] * p_h[13]);
    double x16 = x1 * x5;
    double x17 = x8 * (1 - x16);
    double x18 = p_l[13] * x11 * x9;
    double x19 = (x11 * x11);
    double x20 = b[66] * u[13];
    double x21 = b[69] * x3;
    double x22 = b[68] * x9;
    double x23 = b[67] * x12;

    Eigen::Matrix<double, 2, 2> dF_dx2_T;
    dF_dx2_T << -a[65] * x2 - x0 - x10 * x7 - x11 * x13 - x4 * x6, a[65] * x14 + a[67] * x18 + a[68] * x17 + x0 * x11 + x0 * x7 + x10 * x16 + x13 * x19 + x15 * x4, -b[65] * x2 - x11 * x23 - x20 - x21 * x6 - x22 * x7, b[65] * x14 + b[67] * x18 + b[68] * x17 + x11 * x20 + x15 * x21 + x16 * x22 + x19 * x23 + x20 * x7;
    return dF_dx2_T;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL2_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[61] * u[12];
    double x1 = (p_l[12] * p_l[12]);
    double x2 = p_h[12] * u[12] / x1;
    double x3 = 2 * u[12];
    double x4 = a[64] * x3;
    double x5 = (p_h[12]);
    double x6 = p_l[12] * x5;
    double x7 = p_l[12] / p_h[12];
    double x8 = (u[12] * u[12]);
    double x9 = 2 * x8;
    double x10 = a[63] * x9;
    double x11 = 1 - x7;
    double x12 = p_h[12] * x9;
    double x13 = a[62] * x12;
    double x14 = u[12] / p_l[12];
    double x15 = x1 / (p_h[12] * p_h[12] * p_h[12]);
    double x16 = x1 * x5;
    double x17 = x8 * (1 - x16);
    double x18 = p_l[12] * x11 * x9;
    double x19 = (x11 * x11);
    double x20 = b[61] * u[12];
    double x21 = b[64] * x3;
    double x22 = b[63] * x9;
    double x23 = b[62] * x12;

    Eigen::Matrix<double, 2, 2> dF_dx3_T;
    dF_dx3_T << -a[60] * x2 - x0 - x10 * x7 - x11 * x13 - x4 * x6, a[60] * x14 + a[62] * x18 + a[63] * x17 + x0 * x11 + x0 * x7 + x10 * x16 + x13 * x19 + x15 * x4, -b[60] * x2 - x11 * x23 - x20 - x21 * x6 - x22 * x7, b[60] * x14 + b[62] * x18 + b[63] * x17 + x11 * x20 + x15 * x21 + x16 * x22 + x19 * x23 + x20 * x7;
    return dF_dx3_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL3_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[56] * u[11];
    double x1 = (p_l[11] * p_l[11]);
    double x2 = p_h[11] * u[11] / x1;
    double x3 = 2 * u[11];
    double x4 = a[59] * x3;
    double x5 = (p_h[11]);
    double x6 = p_l[11] * x5;
    double x7 = p_l[11] / p_h[11];
    double x8 = (u[11] * u[11]);
    double x9 = 2 * x8;
    double x10 = a[58] * x9;
    double x11 = 1 - x7;
    double x12 = p_h[11] * x9;
    double x13 = a[57] * x12;
    double x14 = u[11] / p_l[11];
    double x15 = x1 / (p_h[11] * p_h[11] * p_h[11]);
    double x16 = x1 * x5;
    double x17 = x8 * (1 - x16);
    double x18 = p_l[11] * x11 * x9;
    double x19 = (x11 * x11);
    double x20 = b[56] * u[11];
    double x21 = b[59] * x3;
    double x22 = b[58] * x9;
    double x23 = b[57] * x12;
    Eigen::Matrix<double, 2, 2> dF_dx4_T;
    dF_dx4_T << -a[55] * x2 - x0 - x10 * x7 - x11 * x13 - x4 * x6, a[55] * x14 + a[57] * x18 + a[58] * x17 + x0 * x11 + x0 * x7 + x10 * x16 + x13 * x19 + x15 * x4, -b[55] * x2 - x11 * x23 - x20 - x21 * x6 - x22 * x7, b[55] * x14 + b[57] * x18 + b[58] * x17 + x11 * x20 + x15 * x21 + x16 * x22 + x19 * x23 + x20 * x7;
    return dF_dx4_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL4_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[51] * u[10];
    double x1 = (p_l[10] * p_l[10]);
    double x2 = p_h[10] * u[10] / x1;
    double x3 = 2 * u[10];
    double x4 = a[54] * x3;
    double x5 = (p_h[10]);
    double x6 = p_l[10] * x5;
    double x7 = p_l[10] / p_h[10];
    double x8 = (u[10] * u[10]);
    double x9 = 2 * x8;
    double x10 = a[53] * x9;
    double x11 = 1 - x7;
    double x12 = p_h[10] * x9;
    double x13 = a[52] * x12;
    double x14 = u[10] / p_l[10];
    double x15 = x1 / (p_h[10] * p_h[10] * p_h[10]);
    double x16 = x1 * x5;
    double x17 = x8 * (1 - x16);
    double x18 = p_l[10] * x11 * x9;
    double x19 = (x11 * x11);
    double x20 = b[51] * u[10];
    double x21 = b[54] * x3;
    double x22 = b[53] * x9;
    double x23 = b[52] * x12;

    Eigen::Matrix<double, 2, 2> dF_dx5_T;
    dF_dx5_T << -a[50] * x2 - x0 - x10 * x7 - x11 * x13 - x4 * x6, a[50] * x14 + a[52] * x18 + a[53] * x17 + x0 * x11 + x0 * x7 + x10 * x16 + x13 * x19 + x15 * x4, -b[50] * x2 - x11 * x23 - x20 - x21 * x6 - x22 * x7, b[50] * x14 + b[52] * x18 + b[53] * x17 + x11 * x20 + x15 * x21 + x16 * x22 + x19 * x23 + x20 * x7;
    return dF_dx5_T;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[14] / p_l[14];
    double x1 = p_l[14] / p_h[14];
    double x2 = a[71] * u[14];
    double x3 = 2 * u[14];
    double x4 = a[74] * x3;
    double x5 = (p_l[14] * p_l[14]);
    double x6 = x5 / (p_h[14] * p_h[14] * p_h[14]);
    double x7 = 1 - x1;
    double x8 = (p_h[14]);
    double x9 = x5 * x8;
    double x10 = (u[14] * u[14]);
    double x11 = 2 * x10;
    double x12 = a[73] * x11;
    double x13 = x10 * (1 - x9);
    double x14 = a[72] * x11;
    double x15 = p_l[14] * x7;
    double x16 = (x7 * x7);
    double x17 = p_h[14] * x14;
    double x18 = p_h[14] * u[14] / x5;
    double x19 = p_l[14] * x8;
    double x20 = b[71] * u[14];
    double x21 = b[74] * x3;
    double x22 = b[73] * x11;
    double x23 = b[72] * x11;
    double x24 = p_h[14] * x23;
    Eigen::Matrix<double, 2, 2> dF_dx_T;
    dF_dx_T << a[70] * x0 + a[73] * x13 + x1 * x2 + x12 * x9 + x14 * x15 + x16 * x17 + x2 * x7 + x4 * x6, -a[70] * x18 - x1 * x12 - x17 * x7 - x19 * x4 - x2, b[70] * x0 + b[73] * x13 + x1 * x20 + x15 * x23 + x16 * x24 + x20 * x7 + x21 * x6 + x22 * x9, -b[70] * x18 - x1 * x22 - x19 * x21 - x20 - x24 * x7;
    return dF_dx_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH1_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[13] / p_l[13];
    double x1 = p_l[13] / p_h[13];
    double x2 = a[66] * u[13];
    double x3 = 2 * u[13];
    double x4 = a[69] * x3;
    double x5 = (p_l[13] * p_l[13]);
    double x6 = x5 / (p_h[13] * p_h[13] * p_h[13]);
    double x7 = 1 - x1;
    double x8 = (p_h[13]);
    double x9 = x5 * x8;
    double x10 = (u[13] * u[13]);
    double x11 = 2 * x10;
    double x12 = a[68] * x11;
    double x13 = x10 * (1 - x9);
    double x14 = a[67] * x11;
    double x15 = p_l[13] * x7;
    double x16 = (x7 * x7);
    double x17 = p_h[13] * x14;
    double x18 = p_h[13] * u[13] / x5;
    double x19 = p_l[13] * x8;
    double x20 = b[66] * u[13];
    double x21 = b[69] * x3;
    double x22 = b[68] * x11;
    double x23 = b[67] * x11;
    double x24 = p_h[13] * x23;

    Eigen::Matrix<double, 2, 2> dF_dx2_T;
    dF_dx2_T << a[65] * x0 + a[68] * x13 + x1 * x2 + x12 * x9 + x14 * x15 + x16 * x17 + x2 * x7 + x4 * x6, -a[65] * x18 - x1 * x12 - x17 * x7 - x19 * x4 - x2, b[65] * x0 + b[68] * x13 + x1 * x20 + x15 * x23 + x16 * x24 + x20 * x7 + x21 * x6 + x22 * x9, -b[65] * x18 - x1 * x22 - x19 * x21 - x20 - x24 * x7;
    return dF_dx2_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH2_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[12] / p_l[12];
    double x1 = p_l[12] / p_h[12];
    double x2 = a[61] * u[12];
    double x3 = 2 * u[12];
    double x4 = a[64] * x3;
    double x5 = (p_l[12] * p_l[12]);
    double x6 = x5 / (p_h[12] * p_h[12] * p_h[12]);
    double x7 = 1 - x1;
    double x8 = (p_h[12]);
    double x9 = x5 * x8;
    double x10 = (u[12] * u[12]);
    double x11 = 2 * x10;
    double x12 = a[63] * x11;
    double x13 = x10 * (1 - x9);
    double x14 = a[62] * x11;
    double x15 = p_l[12] * x7;
    double x16 = (x7 * x7);
    double x17 = p_h[12] * x14;
    double x18 = p_h[12] * u[12] / x5;
    double x19 = p_l[12] * x8;
    double x20 = b[61] * u[12];
    double x21 = b[64] * x3;
    double x22 = b[63] * x11;
    double x23 = b[62] * x11;
    double x24 = p_h[12] * x23;
    Eigen::Matrix<double, 2, 2> dF_dx3_T;
    dF_dx3_T << a[60] * x0 + a[63] * x13 + x1 * x2 + x12 * x9 + x14 * x15 + x16 * x17 + x2 * x7 + x4 * x6, -a[60] * x18 - x1 * x12 - x17 * x7 - x19 * x4 - x2, b[60] * x0 + b[63] * x13 + x1 * x20 + x15 * x23 + x16 * x24 + x20 * x7 + x21 * x6 + x22 * x9, -b[60] * x18 - x1 * x22 - x19 * x21 - x20 - x24 * x7;
    return dF_dx3_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH3_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[11] / p_l[11];
    double x1 = p_l[11] / p_h[11];
    double x2 = a[56] * u[11];
    double x3 = 2 * u[11];
    double x4 = a[59] * x3;
    double x5 = (p_l[11] * p_l[11]);
    double x6 = x5 / (p_h[11] * p_h[11] * p_h[11]);
    double x7 = 1 - x1;
    double x8 = (p_h[11]);
    double x9 = x5 * x8;
    double x10 = (u[11] * u[11]);
    double x11 = 2 * x10;
    double x12 = a[58] * x11;
    double x13 = x10 * (1 - x9);
    double x14 = a[57] * x11;
    double x15 = p_l[11] * x7;
    double x16 = (x7 * x7);
    double x17 = p_h[11] * x14;
    double x18 = p_h[11] * u[11] / x5;
    double x19 = p_l[11] * x8;
    double x20 = b[56] * u[11];
    double x21 = b[59] * x3;
    double x22 = b[58] * x11;
    double x23 = b[57] * x11;
    double x24 = p_h[11] * x23;

    Eigen::Matrix<double, 2, 2> dF_dx4_T;
    dF_dx4_T << a[55] * x0 + a[58] * x13 + x1 * x2 + x12 * x9 + x14 * x15 + x16 * x17 + x2 * x7 + x4 * x6, -a[55] * x18 - x1 * x12 - x17 * x7 - x19 * x4 - x2, b[55] * x0 + b[58] * x13 + x1 * x20 + x15 * x23 + x16 * x24 + x20 * x7 + x21 * x6 + x22 * x9, -b[55] * x18 - x1 * x22 - x19 * x21 - x20 - x24 * x7;
    return dF_dx4_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH4_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[10] / p_l[10];
    double x1 = p_l[10] / p_h[10];
    double x2 = a[51] * u[10];
    double x3 = 2 * u[10];
    double x4 = a[54] * x3;
    double x5 = (p_l[10] * p_l[10]);
    double x6 = x5 / (p_h[10] * p_h[10] * p_h[10]);
    double x7 = 1 - x1;
    double x8 = (p_h[10]);
    double x9 = x5 * x8;
    double x10 = (u[10] * u[10]);
    double x11 = 2 * x10;
    double x12 = a[53] * x11;
    double x13 = x10 * (1 - x9);
    double x14 = a[52] * x11;
    double x15 = p_l[10] * x7;
    double x16 = (x7 * x7);
    double x17 = p_h[10] * x14;
    double x18 = p_h[10] * u[10] / x5;
    double x19 = p_l[10] * x8;
    double x20 = b[51] * u[10];
    double x21 = b[54] * x3;
    double x22 = b[53] * x11;
    double x23 = b[52] * x11;
    double x24 = p_h[10] * x23;
    Eigen::Matrix<double, 2, 2> dF_dx5_T;
    dF_dx5_T << a[50] * x0 + a[53] * x13 + x1 * x2 + x12 * x9 + x14 * x15 + x16 * x17 + x2 * x7 + x4 * x6, -a[50] * x18 - x1 * x12 - x17 * x7 - x19 * x4 - x2, b[50] * x0 + b[53] * x13 + x1 * x20 + x15 * x23 + x16 * x24 + x20 * x7 + x21 * x6 + x22 * x9, -b[50] * x18 - x1 * x22 - x19 * x21 - x20 - x24 * x7;
    return dF_dx5_T;
}

Eigen::Matrix<double, 2, 1> MPC::Update_dF_du_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[14] / p_l[14];
    double x1 = 1 - p_l[14] / p_h[14];
    double x2 = p_h[14] * x1;
    double x3 = (p_h[14] * p_h[14]);
    double x4 = -(p_l[14] * p_l[14]) / x3 + 1;
    double x5 = 2 * u[14];
    double x6 = p_h[14] * x4 * x5;
    double x7 = (x1 * x1) * x3 * x5;
    Eigen::Matrix<double, 2, 1> dF_du_T;
    dF_du_T << a[70] * x0 + a[71] * x2 + a[72] * x7 + a[73] * x6 + a[74] * x4, b[70] * x0 + b[71] * x2 + b[72] * x7 + b[73] * x6 + b[74] * x4;
    return dF_du_T;
}
Eigen::Matrix<double, 2, 1> MPC::Update_dF_du1_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[13] / p_l[13];
    double x1 = 1 - p_l[13] / p_h[13];
    double x2 = p_h[13] * x1;
    double x3 = (p_h[13] * p_h[13]);
    double x4 = -(p_l[13] * p_l[13]) / x3 + 1;
    double x5 = 2 * u[13];
    double x6 = p_h[13] * x4 * x5;
    double x7 = (x1 * x1) * x3 * x5;
    Eigen::Matrix<double, 2, 1> dF_du2_T;
    dF_du2_T << a[65] * x0 + a[66] * x2 + a[67] * x7 + a[68] * x6 + a[69] * x4, b[65] * x0 + b[66] * x2 + b[67] * x7 + b[68] * x6 + b[69] * x4;
    return dF_du2_T;
}

Eigen::Matrix<double, 2, 1> MPC::Update_dF_du2_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[12] / p_l[12];
    double x1 = 1 - p_l[12] / p_h[12];
    double x2 = p_h[12] * x1;
    double x3 = (p_h[12] * p_h[12]);
    double x4 = -(p_l[12] * p_l[12]) / x3 + 1;
    double x5 = 2 * u[12];
    double x6 = p_h[12] * x4 * x5;
    double x7 = (x1 * x1) * x3 * x5;
    Eigen::Matrix<double, 2, 1> dF_du3_T;
    dF_du3_T << a[60] * x0 + a[61] * x2 + a[62] * x7 + a[63] * x6 + a[64] * x4, b[60] * x0 + b[61] * x2 + b[62] * x7 + b[63] * x6 + b[64] * x4;
    return dF_du3_T;
}

Eigen::Matrix<double, 2, 1> MPC::Update_dF_du3_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[11] / p_l[11];
    double x1 = 1 - p_l[11] / p_h[11];
    double x2 = p_h[11] * x1;
    double x3 = (p_h[11] * p_h[11]);
    double x4 = -(p_l[11] * p_l[11]) / x3 + 1;
    double x5 = 2 * u[11];
    double x6 = p_h[11] * x4 * x5;
    double x7 = (x1 * x1) * x3 * x5;
    Eigen::Matrix<double, 2, 1> dF_du4_T;
    dF_du4_T << a[55] * x0 + a[56] * x2 + a[57] * x7 + a[58] * x6 + a[59] * x4, b[55] * x0 + b[56] * x2 + b[57] * x7 + b[58] * x6 + b[59] * x4;
    return dF_du4_T;
}
Eigen::Matrix<double, 2, 1> MPC::Update_dF_du4_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[10] / p_l[10];
    double x1 = 1 - p_l[10] / p_h[10];
    double x2 = p_h[10] * x1;
    double x3 = (p_h[10] * p_h[10]);
    double x4 = -(p_l[10] * p_l[10]) / x3 + 1;
    double x5 = 2 * u[10];
    double x6 = p_h[10] * x4 * x5;
    double x7 = (x1 * x1) * x3 * x5;
    Eigen::Matrix<double, 2, 1> dF_du5_T;
    dF_du5_T << a[50] * x0 + a[51] * x2 + a[52] * x7 + a[53] * x6 + a[54] * x4, b[50] * x0 + b[51] * x2 + b[52] * x7 + b[53] * x6 + b[54] * x4;
    return dF_du5_T;
}

int MPC::DutyCalculate(bool increase_pre, std::array<float, MPC_TIME_HORIZON> y_des, double scale)
{

    this->y_des1 = y_des[0];
    this->y_des2 = y_des[1];
    this->y_des3 = y_des[2];
    this->y_des4 = y_des[3];
    this->y_des5 = y_des[4];

    Eigen::Matrix<double, 2, 1> f_p1, f_p2, f_p3, f_p4, f_p5;
    Eigen::Matrix<double, 2, 1> df_dun_p1_T, df_dun_p2_T, df_dun_p3_T, df_dun_p4_T, df_dun_p5_T;
    Eigen::Matrix<double, 2, 1> df_dun1_p2_T, df_dun1_p3_T, df_dun1_p4_T, df_dun1_p5_T;
    Eigen::Matrix<double, 2, 1> df_dun2_p3_T, df_dun2_p4_T, df_dun2_p5_T;
    Eigen::Matrix<double, 2, 1> df_dun3_p4_T, df_dun3_p5_T;
    Eigen::Matrix<double, 2, 1> df_dun4_p5_T;

    Eigen::Matrix<double, 2, 2> df_dxn_p2_T, df_dxn_p3_T, df_dxn_p4_T, df_dxn_p5_T;
    Eigen::Matrix<double, 2, 2> df_dxn1_p3_T, df_dxn1_p4_T, df_dxn1_p5_T;
    Eigen::Matrix<double, 2, 2> df_dxn2_p4_T, df_dxn2_p5_T;
    Eigen::Matrix<double, 2, 2> df_dxn3_p5_T;

    Eigen::Matrix<double, 2, 1> F_offset;
    F_offset << *(this->p_tank_his.end() - 1), *(this->p_set_his.end() - 1);

    double *p_h;
    double *p_l;
    std::array<double, MPC_STATE_NUM> *cur_a;
    std::array<double, MPC_STATE_NUM> *cur_b;

    if (increase_pre)
    {
        // if we are increasing the pressure

        p_h = this->p_tank_his.begin();
        p_l = this->p_set_his.begin();
        cur_a = &this->ah;
        cur_b = &this->bh;

        // this->cur_F = scale * this->UpdateF(this->p_tank_his.begin(), this->p_set_his.begin(), this->u_his.begin(), this->ah, this->bh) + F_offset;

        // f_p1 = scale * this->UpdateF(this->p_tank_his.begin(), this->p_set_his.begin(), this->u_his.begin() + 1, this->ah, this->bh) + F_offset;
        // f_p2 = scale * this->UpdateF(this->p_tank_his.begin() + 1, this->p_set_his.begin() + 1, this->u_his.begin() + 2, this->ah, this->bh) + F_offset;
        // f_p3 = scale * this->UpdateF(this->p_tank_his.begin() + 2, this->p_set_his.begin() + 2, this->u_his.begin() + 3, this->ah, this->bh) + F_offset;

        // df_dun_p1_T = scale * this->Update_dF_du_T(this->p_tank_his.begin(), this->p_set_his.begin(), this->u_his.begin() + 1, this->ah, this->bh);
        // df_dun_p2_T = scale * this->Update_dF_du_T(this->p_tank_his.begin() + 1, this->p_set_his.begin() + 1, this->u_his.begin() + 2, this->ah, this->bh);
        // df_dun_p3_T = scale * this->Update_dF_du_T(this->p_tank_his.begin() + 2, this->p_set_his.begin() + 2, this->u_his.begin() + 3, this->ah, this->bh);

        // df_dun1_p2_T = scale * this->Update_dF_du1_T(this->p_tank_his.begin() + 1, this->p_set_his.begin() + 1, this->u_his.begin() + 2, this->ah, this->bh);
        // df_dun1_p3_T = scale * this->Update_dF_du1_T(this->p_tank_his.begin() + 2, this->p_set_his.begin() + 2, this->u_his.begin() + 3, this->ah, this->bh);
        // df_dun2_p3_T = scale * this->Update_dF_du2_T(this->p_tank_his.begin() + 2, this->p_set_his.begin() + 2, this->u_his.begin() + 3, this->ah, this->bh);

        // df_dxn_p2_T = scale * this->Update_dF_dxH_T(this->p_tank_his.begin() + 1, this->p_set_his.begin() + 1, this->u_his.begin() + 2, this->ah, this->bh);
        // df_dxn_p3_T = scale * this->Update_dF_dxH_T(this->p_tank_his.begin() + 2, this->p_set_his.begin() + 2, this->u_his.begin() + 3, this->ah, this->bh);

        // df_dxn1_p3_T = this->Update_dF_dxH1_T(this->p_tank_his.begin() + 2, this->p_set_his.begin() + 2, this->u_his.begin() + 3, this->ah, this->bh);
    }
    else
    {
        p_h = this->p_set_his.begin();
        p_l = this->p_tank_his.begin();
        cur_a = &this->al;
        cur_b = &this->bl;
        // this->cur_F = this->UpdateF(this->p_set_his.begin(), this->p_tank_his.begin(), this->u_his.begin(), this->al, this->bl) + F_offset;

        // f_p1 = this->UpdateF(this->p_set_his.begin(), this->p_tank_his.begin(), this->u_his.begin() + 1, this->al, this->bl) + F_offset;
        // f_p2 = this->UpdateF(this->p_set_his.begin() + 1, this->p_tank_his.begin() + 1, this->u_his.begin() + 2, this->al, this->bl) + F_offset;
        // f_p3 = this->UpdateF(this->p_set_his.begin() + 2, this->p_tank_his.begin() + 2, this->u_his.begin() + 3, this->al, this->bl) + F_offset;

        // df_dun_p1_T = this->Update_dF_du_T(this->p_set_his.begin(), this->p_tank_his.begin(), this->u_his.begin() + 1, this->al, this->bl);
        // df_dun_p2_T = this->Update_dF_du_T(this->p_set_his.begin() + 1, this->p_tank_his.begin() + 1, this->u_his.begin() + 2, this->al, this->bl);
        // df_dun_p3_T = this->Update_dF_du_T(this->p_set_his.begin() + 2, this->p_tank_his.begin() + 2, this->u_his.begin() + 3, this->al, this->bl);

        // df_dun1_p2_T = this->Update_dF_du1_T(this->p_set_his.begin() + 1, this->p_tank_his.begin() + 1, this->u_his.begin() + 2, this->al, this->bl);
        // df_dun1_p3_T = this->Update_dF_du1_T(this->p_set_his.begin() + 2, this->p_tank_his.begin() + 2, this->u_his.begin() + 3, this->al, this->bl);
        // df_dun2_p3_T = this->Update_dF_du2_T(this->p_set_his.begin() + 2, this->p_tank_his.begin() + 2, this->u_his.begin() + 3, this->al, this->bl);

        // df_dxn_p2_T = this->Update_dF_dxL_T(this->p_set_his.begin() + 1, this->p_tank_his.begin() + 1, this->u_his.begin() + 2, this->al, this->bl);
        // df_dxn_p3_T = this->Update_dF_dxL_T(this->p_set_his.begin() + 2, this->p_tank_his.begin() + 2, this->u_his.begin() + 3, this->al, this->bl);

        // df_dxn1_p3_T = this->Update_dF_dxL1_T(this->p_set_his.begin() + 2, this->p_tank_his.begin() + 2, this->u_his.begin() + 3, this->al, this->bl);
    }

    this->cur_dF = scale * this->UpdateF(p_h, p_l, this->u_his.begin(), *cur_a, *cur_b);
    this->cur_F = this->cur_dF + F_offset;

    f_p1 = scale * this->UpdateF(p_h, p_l, this->u_his.begin() + 1, *cur_a, *cur_b) + F_offset;
    f_p2 = scale * this->UpdateF(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b) + F_offset;
    f_p3 = scale * this->UpdateF(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b) + F_offset;
    f_p4 = scale * this->UpdateF(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b) + F_offset;
    f_p5 = scale * this->UpdateF(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b) + F_offset;

    df_dun_p1_T = scale * this->Update_dF_du_T(p_h, p_l, this->u_his.begin() + 1, *cur_a, *cur_b);
    df_dun_p2_T = scale * this->Update_dF_du_T(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun_p3_T = scale * this->Update_dF_du_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun_p4_T = scale * this->Update_dF_du_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun_p5_T = scale * this->Update_dF_du_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dun1_p2_T = scale * this->Update_dF_du1_T(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun1_p3_T = scale * this->Update_dF_du1_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun1_p4_T = scale * this->Update_dF_du1_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun1_p5_T = scale * this->Update_dF_du1_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dun2_p3_T = scale * this->Update_dF_du2_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun2_p4_T = scale * this->Update_dF_du2_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun2_p5_T = scale * this->Update_dF_du2_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dun3_p4_T = scale * this->Update_dF_du3_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun3_p5_T = scale * this->Update_dF_du3_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);

    df_dun4_p5_T = scale * this->Update_dF_du4_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dxn_p2_T = scale * this->Update_dF_dxH_T(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dxn_p3_T = scale * this->Update_dF_dxH_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn_p4_T = scale * this->Update_dF_dxH_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn_p5_T = scale * this->Update_dF_dxH_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dxn1_p3_T = scale * this->Update_dF_dxH1_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn1_p4_T = scale * this->Update_dF_dxH1_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn1_p5_T = scale * this->Update_dF_dxH1_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dxn2_p4_T = scale * this->Update_dF_dxH2_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn2_p5_T = scale * this->Update_dF_dxH2_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    df_dxn3_p5_T = scale * this->Update_dF_dxH3_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);

    Eigen::Matrix<double, 2, 1> x_bar;
    Eigen::Matrix<double, 2, 1> zero_col(0, 0);

    x_bar << this->p_tank_his[MPC_DELAY], this->p_set_his[MPC_DELAY];

    Eigen::Matrix<double, 2, 1> A1 = f_p1 - df_dun_p1_T * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B1;
    B1 << df_dun_p1_T, zero_col, zero_col, zero_col, zero_col;

    Eigen::Matrix<double, 2, 1> A2 = f_p2 + df_dxn_p2_T * A1 - (df_dun_p2_T + df_dun1_p2_T) * MPC::kUBar - df_dxn_p2_T * x_bar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B2;
    B2 << df_dun1_p2_T, df_dun_p2_T, zero_col, zero_col, zero_col;
    B2 += df_dxn_p2_T * B1;

    Eigen::Matrix<double, 2, 1> A3 = f_p3 + df_dxn_p3_T * (A2 - x_bar) + df_dxn1_p3_T * (A1 - x_bar) - (df_dun_p3_T + df_dun1_p3_T + df_dun2_p3_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B3;
    B3 << df_dun2_p3_T, df_dun1_p3_T, df_dun_p3_T, zero_col, zero_col;
    B3 += df_dxn_p3_T * B2 + df_dxn1_p3_T * B1;

    Eigen::Matrix<double, 2, 1> A4 = f_p4 + df_dxn_p4_T * (A3 - x_bar) + df_dxn1_p4_T * (A2 - x_bar) + df_dxn2_p4_T * (A1 - x_bar) - (df_dun_p4_T + df_dun1_p4_T + df_dun2_p4_T + df_dun3_p4_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B4;
    B4 << df_dun3_p4_T, df_dun2_p4_T, df_dun1_p4_T, df_dun_p4_T, zero_col;
    B4 += df_dxn_p4_T * B3 + df_dxn1_p4_T * B2 + df_dxn2_p4_T * B1;

    Eigen::Matrix<double, 2, 1> A5 = f_p5 + df_dxn_p5_T * (A4 - x_bar) + df_dxn1_p5_T * (A3 - x_bar) + df_dxn2_p5_T * (A2 - x_bar) + df_dxn3_p5_T * (A1 - x_bar) - (df_dun_p5_T + df_dun1_p5_T + df_dun2_p5_T + df_dun3_p5_T + df_dun4_p5_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B5;
    B5 << df_dun4_p5_T, df_dun3_p5_T, df_dun2_p5_T, df_dun1_p5_T, df_dun_p5_T;
    B5 += df_dxn_p5_T * B4 + df_dxn1_p5_T * B3 + df_dxn2_p5_T * B2 + df_dxn3_p5_T * B1;

    // combine all matrices
    Eigen::Matrix<double, 1, 2> H(0, 1);
    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> A_all;
    Eigen::Matrix<double, MPC_TIME_HORIZON, MPC_TIME_HORIZON> B_all;
    A_all << H * A1, H * A2, H * A3, H * A4, H * A5;
    B_all << H * B1, H * B2, H * B3, H * B4, H * B5;
    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> y_des_vec;
    y_des_vec << y_des[0], y_des[1], y_des[2], y_des[3], y_des[4];
    Eigen::Matrix<double, MPC_TIME_HORIZON, MPC_TIME_HORIZON> P_mat = B_all.transpose() * B_all;
    Eigen::Matrix<double, 1, MPC_TIME_HORIZON> q_mat = -1 * y_des_vec.transpose() * B_all + A_all.transpose() * B_all;

    // std::cout<<"pmat: "<<P_mat<<std::endl;
    // std::cout<<"qmat: "<<q_mat<<std::endl;
    // std::cout<<"ydes: "<<y_des[0]<<','<<y_des[1]<<','<<y_des[2]<<std::endl;
    // std::cout<<"ycur: "<<this->p_set_his[MPC_DELAY]<<std::endl;
    // form the problem into osqp
    c_float P_x[(MPC_TIME_HORIZON + 1) * MPC_TIME_HORIZON / 2] = {P_mat.coeff(0, 0),
                                                                  P_mat.coeff(0, 1), P_mat.coeff(1, 1),
                                                                  P_mat.coeff(0, 2), P_mat.coeff(1, 2), P_mat.coeff(2, 2),
                                                                  P_mat.coeff(0, 3), P_mat.coeff(1, 3), P_mat.coeff(2, 3), P_mat.coeff(3, 3),
                                                                  P_mat.coeff(0, 4), P_mat.coeff(1, 4), P_mat.coeff(2, 4), P_mat.coeff(3, 4), P_mat.coeff(4, 4)};
    c_int P_i[(MPC_TIME_HORIZON + 1) * MPC_TIME_HORIZON / 2] = {0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 3, 4};
    c_int P_p[MPC_TIME_HORIZON + 1] = {0, 1, 3, 6, 10, 15};
    c_int P_nnz = (MPC_TIME_HORIZON + 1) * MPC_TIME_HORIZON / 2;
    c_float q[MPC_TIME_HORIZON] = {q_mat.coeff(0, 0), q_mat.coeff(0, 1), q_mat.coeff(0, 2), q_mat.coeff(0, 3), q_mat.coeff(0, 4)};

    c_int A_nnz = MPC_TIME_HORIZON;
    c_float A_x[MPC_TIME_HORIZON] = {1, 1, 1, 1, 1};
    c_int A_i[MPC_TIME_HORIZON] = {0, 1, 2, 3, 4};
    c_int A_p[MPC_TIME_HORIZON + 1] = {0, 1, 2, 3, 4, 5};
    c_float l[MPC_TIME_HORIZON] = {0.15, 0, 0, 0, 0};
    c_float u[MPC_TIME_HORIZON] = {1, 1, 1, 1, 1};
    c_int n = MPC_TIME_HORIZON;
    c_int m = MPC_TIME_HORIZON;
    // populate the data
    this->osqp_data->n = n;
    this->osqp_data->m = m;
    this->osqp_data->P = csc_matrix(n, n, P_nnz, P_x, P_i, P_p);
    this->osqp_data->q = q;
    this->osqp_data->A = csc_matrix(m, n, A_nnz, A_x, A_i, A_p);
    this->osqp_data->l = l;
    this->osqp_data->u = u;

    // solve osqp
    c_int exit_flag = osqp_setup(&this->work, this->osqp_data.get(), this->osqp_settings.get());
    osqp_solve(this->work);

    this->u_n = *(this->work->solution->x);
    this->u_n1 = *(this->work->solution->x + 1);
    this->u_n2 = *(this->work->solution->x + 2);
    this->u_n3 = *(this->work->solution->x + 3);
    this->u_n4 = *(this->work->solution->x + 4);

    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> u_vec;
    u_vec << this->u_n, this->u_n1, this->u_n2, this->u_n3, this->u_n4;
    // std::cout<<"u vec: "<<u_vec<<std::endl;

    this->x_n1 = A1 + B1 * u_vec;
    this->x_n2 = A2 + B2 * u_vec;
    this->x_n3 = A3 + B3 * u_vec;
    this->x_n4 = A4 + B4 * u_vec;
    this->x_n5 = A5 + B5 * u_vec;
    // std::cout<<"u_n: "<<this->u_n<<", u_1: "<<this->u_n1<<", u_2: "<<this->u_n2<<std::endl;
    // std::cout<<"x_n1: "<<this->x_n1.coeff(1,0)<<", x_n2: "<<this->x_n2.coeff(1,0)<<", x_n3: "<<this->x_n3.coeff(1,0)<<std::endl;

    // auto res_err = u_vec.transpose()*P_mat*u_vec+2*q_mat*u_vec;
    // std::cout<<"residual: "<<res_err*2<<std::endl;
    // auto err = y_des_vec-A_all-B_all*u_vec;
    // auto res_err2 = err.norm();
    // std::cout<<"real residual: "<<res_err2<<std::endl;
    return (int)(this->u_n * 100 + 0.5);
}

int MPC::GetPreControl(const double &p_des, const double &ps, const double &pt, double scale)
{

    double p_diff = (p_des - ps); // we scaled the p_diff with the assumption that pressure will have the momentum to go
    // std::cout<<"p_diff: "<<p_diff<<std::endl;
    if (std::abs(p_diff) > 100) // 640 is 2 psi
    {                           // if desired pressure has 1 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias

        // double lb;
        // if(std::abs(pt-ps)<1310){ //if the difference is less than 10 psi, we can operate the valve with lower duty
        //     lb = 10.0;
        // }
        // else if(std::abs(pt-ps)<2621){ //20 psi
        //     lb = 20.0;
        // }

        // else{
        //     lb=60.0;
        // }

        // double lb = 20; //set the lower bound 20 duty
        float p_des_scale = ((float)p_des - 3297.312) / 65536.0;
        std::array<float, MPC_TIME_HORIZON> y_des{p_des_scale, p_des_scale, p_des_scale, p_des_scale, p_des_scale};
        // std::cout<<"p_diff: "<<p_diff/65536<<std::endl;
        this->UpdateHistory(ps, pt);

        // std::cout<<"current p_des scale: "<<p_des_scale<<std::endl;
        // std::cout<<"current p_cur scale: "<<this->p_set_his[MPC_TIME_HORIZON]<<std::endl;

        int ideal_duty = 0;
        if ((p_des > ps) & (pt > ps) & (pt > p_des))
        {
            // increasing pressure
            //  std::cout<<"increase\n";

            ideal_duty = this->DutyCalculate(true, y_des, scale);
        }
        else if ((p_des < ps) & (ps > pt) & (p_des > pt))
        {
            // decreasing pressure
            //  std::cout<<"decrease\n";
            ideal_duty = this->DutyCalculate(false, y_des, scale);
        }
        else
        {

            return 0;
        }

        // std::cout<<"real p_diff: "<<this->Phi.coeff(1,0)*65536<<std::endl;

        // std::cout<<std::endl;
        // std::cout << "ideal duty: " << ideal_duty << std::endl;
        // std::cout << "control p_diff: " << (this->alpha.coeff(1, 0) + this->B.coeff(1, 0) * ideal_duty/100) * 65536 << std::endl;
        // std::cout << "des pdiff: " << p_diff << std::endl;

        if (ideal_duty <= 5)
        {
            return 0;
        }
        else if (ideal_duty < 15 && ideal_duty > 5)
        {
            // return 0;
            return 15;
        }
        else if (ideal_duty > 100)
            return 100;
        else
        {
            return ideal_duty;
        }
    }
    else
    {
        // this->cur_F << 0, 0;
        this->cur_dF << 0, 0;
        this->u_n = 0;
        this->u_n1 = 0;
        this->u_n2 = 0;
        return 0;
    }
}

void MPC::PushMeas(const double p_tank, const double p_set, const uint8_t duty)
{
    this->p_tank_mem[this->meas_idx] = ((double)p_tank - 3297.312) / 65536.0; // the substraction is to remove the 0.5 V pressure sensor bias and add 1 atm to the equation
    this->p_set_mem[this->meas_idx] = ((double)p_set - 3297.312) / 65536.0;   // in the lasso regression, we have proved it increases the testing accuracy to stable 90% up
    this->u_mem[this->meas_idx] = ((double)duty) / 100;
    this->meas_idx++;
    this->meas_idx %= MPC_DELAY;

    this->cur_F << ((double)p_tank - 3297.312) / 65536.0, ((double)p_set - 3297.312) / 65536.0;
}
void MPC::RecData()
{
    this->mpc_rec.PushData(
        std::array<double, 19>{this->cur_F.coeff(0, 0), this->cur_F.coeff(1, 0), this->cur_dF.coeff(0, 0), this->cur_dF.coeff(1, 0), this->u_n, this->u_n1, this->u_n2, this->u_n3, this->u_n4, this->x_n1.coeff(1, 0), this->x_n2.coeff(1, 0), this->x_n3.coeff(1, 0), this->x_n4.coeff(1, 0), this->x_n5.coeff(1, 0), this->y_des1, this->y_des2, this->y_des3, this->y_des4, this->y_des5});
}

void MPC::UpdateHistory(double p_set, double p_tank)
{
    std::memset(this->p_tank_his.begin(), 0, this->p_tank_his.size());
    std::memset(this->p_set_his.begin(), 0, this->p_set_his.size());
    std::memset(this->u_his.begin(), 0, this->u_his.size());

    for (int i = 0; i < MPC_DELAY; i++)
    {
        this->p_tank_his[i] = this->p_tank_mem[(this->meas_idx + i) % MPC_DELAY];
        this->p_set_his[i] = this->p_set_mem[(this->meas_idx + i) % MPC_DELAY];
        this->u_his[i] = this->u_mem[(this->meas_idx + i) % MPC_DELAY];
    }

    for (int i = 0; i < MPC_TIME_HORIZON; i++)
    {
        this->p_tank_his[i + MPC_DELAY] = (p_tank - 3297.312) / 65536;
        this->p_set_his[i + MPC_DELAY] = (p_set - 3297.312) / 65536;
        this->u_his[i + MPC_DELAY] = MPC::kUBar; // use the lower bound first, in case the previous duty was 0
    }

    // std::cout<<"mem values: "<<this->p_tank_his[0]<<std::endl;
}

// double MPC::GetLenLinear_mm(double pos){ //TODO: this only fit linear case, need to do nonlinear equation when using it on the exoskeleton
//     return pos*this->volume_slope_6in+this->volume_intercept_6in; //152.4 is 6 in cylinder max length
//                                                                                //TODO: need to consider 5" cylinder
//                                                                                //6.71 is just the offset of linear encoder
// }

// double MPC::GetExternalForce(double pre_ext,double pre_flex,double x){
//     //return the external force in newton

//     double cur_delta_x =this->max_pos-x;
//     if((cur_delta_x-4700)>this->cur_max_spring_compress/this->volume_slope_6in){ //It turned out the the spring start to compress earlier, the 4700 is an experimental value

//         return (pre_ext-pre_flex)*2.1547177056884764e-05*this->piston_area-this->fric_coeff*this->pos_diff; //unit newton
//         // return (pre/65536*4.096-0.5)/4*200*0.31-0.001*this->pos_diff;
//     }
//     else{
//         return (this->max_pos-x)*this->volume_slope_6in*this->spring_k;// unit: newton
//         // return (this->max_pos-x)*0.0006351973436310972*this->spring_k/25.4;
//     }

// }

// void MPC::SetCylinderMaxPos(){
// this->max_pos = this->cur_pos;
// }
// double MPC::GetCylinderScale(double pre,double pos) //get the (cylinder length)/(max cylinder length)
// {

//     double cur_pos_mm = this->GetLenLinear_mm(pos);
//     double cur_len = this->max_len_mm - cur_pos_mm-4700*this->volume_slope_6in;
//     if(cur_len<this->cur_max_spring_compress){

//         return (cur_pos_mm-this->cur_max_spring_compress)/this->max_len_mm;
//     }
//     else{
//         return 1;
//     }

// }

// void MPC::GetImpControl(const double& imp_des, const double& p_ext,const double& p_flex, const double& p_tank, const double& pos,double scale,int& joint_val_duty,int& joint_bal_duty,int& tank_duty){
//     //the impedance controller will use the current velocity to estimate the displacement

//     //steps:
//     //       1. calculate desired force based on current position
//     //       2. calculate desired pressure based on current velocity and desired force
//     //       3. command desired pressure

//     // if((this->pos_diff*this->volume_slope_6in<0.01)&&(this->pos_diff*this->volume_slope_6in>-0.01)) return 0; //if it does not move more than 0.5mm, no control

//     double cur_delta_x = this->max_len_mm-this->GetLenLinear_mm(pos);
//     double des_force = imp_des*cur_delta_x;
//     double des_pre = (des_force+this->fric_coeff*this->pos_diff)/this->piston_area/2.1547177056884764e-05+8000;

//     // double p_des = (imp_des*this->pos_diff*this->volume_slope_6in)/this->piston_area*4.641208782079999e+04+p_cur;  //N/mm2 * in2/lbf * 4/200 * 2^16/4.096

//     joint_val_duty= this->GetPreControl(des_pre,p_ext,p_tank,scale);

// }
