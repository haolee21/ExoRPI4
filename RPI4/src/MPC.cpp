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
    double x2 = p_h[15] * u[15];
    double x3 = x2 / p_l[15];
    double x4 = p_h[16] * u[16];
    double x5 = x4 / p_l[16];
    double x6 = p_h[17] * u[17];
    double x7 = x6 / p_l[17];
    double x8 = p_h[18] * u[18];
    double x9 = x8 / p_l[18];
    double x10 = p_h[19] * u[19];
    double x11 = x10 / p_l[19];
    double x12 = p_h[20] * u[20];
    double x13 = x12 / p_l[20];
    double x14 = p_h[21] * u[21];
    double x15 = x14 / p_l[21];
    double x16 = p_h[2] * u[2];
    double x17 = x16 / p_l[2];
    double x18 = p_h[22] * u[22];
    double x19 = x18 / p_l[22];
    double x20 = p_h[23] * u[23];
    double x21 = x20 / p_l[23];
    double x22 = p_h[24] * u[24];
    double x23 = x22 / p_l[24];
    double x24 = p_h[3] * u[3];
    double x25 = x24 / p_l[3];
    double x26 = p_h[4] * u[4];
    double x27 = x26 / p_l[4];
    double x28 = p_h[5] * u[5];
    double x29 = x28 / p_l[5];
    double x30 = p_h[6] * u[6];
    double x31 = x30 / p_l[6];
    double x32 = p_h[7] * u[7];
    double x33 = x32 / p_l[7];
    double x34 = p_h[8] * u[8];
    double x35 = x34 / p_l[8];
    double x36 = p_h[9] * u[9];
    double x37 = x36 / p_l[9];
    double x38 = p_h[10] * u[10];
    double x39 = x38 / p_l[10];
    double x40 = p_h[11] * u[11];
    double x41 = x40 / p_l[11];
    double x42 = p_h[1] * u[1];
    double x43 = x42 / p_l[1];
    double x44 = p_h[12] * u[12];
    double x45 = x44 / p_l[12];
    double x46 = p_h[13] * u[13];
    double x47 = x46 / p_l[13];
    double x48 = p_h[14] * u[14];
    double x49 = x48 / p_l[14];
    double x50 = 1 - p_l[15] / p_h[15];
    double x51 = x2 * x50;
    double x52 = 1 - p_l[16] / p_h[16];
    double x53 = x4 * x52;
    double x54 = 1 - p_l[17] / p_h[17];
    double x55 = x54 * x6;
    double x56 = 1 - p_l[18] / p_h[18];
    double x57 = x56 * x8;
    double x58 = 1 - p_l[19] / p_h[19];
    double x59 = x10 * x58;
    double x60 = 1 - p_l[20] / p_h[20];
    double x61 = x12 * x60;
    double x62 = 1 - p_l[21] / p_h[21];
    double x63 = x14 * x62;
    double x64 = 1 - p_l[22] / p_h[22];
    double x65 = x18 * x64;
    double x66 = 1 - p_l[2] / p_h[2];
    double x67 = x16 * x66;
    double x68 = 1 - p_l[23] / p_h[23];
    double x69 = x20 * x68;
    double x70 = 1 - p_l[24] / p_h[24];
    double x71 = x22 * x70;
    double x72 = 1 - p_l[0] / p_h[0];
    double x73 = x0 * x72;
    double x74 = 1 - p_l[3] / p_h[3];
    double x75 = x24 * x74;
    double x76 = 1 - p_l[4] / p_h[4];
    double x77 = x26 * x76;
    double x78 = 1 - p_l[5] / p_h[5];
    double x79 = x28 * x78;
    double x80 = 1 - p_l[6] / p_h[6];
    double x81 = x30 * x80;
    double x82 = 1 - p_l[7] / p_h[7];
    double x83 = x32 * x82;
    double x84 = 1 - p_l[8] / p_h[8];
    double x85 = x34 * x84;
    double x86 = 1 - p_l[9] / p_h[9];
    double x87 = x36 * x86;
    double x88 = 1 - p_l[10] / p_h[10];
    double x89 = x38 * x88;
    double x90 = 1 - p_l[11] / p_h[11];
    double x91 = x40 * x90;
    double x92 = 1 - p_l[12] / p_h[12];
    double x93 = x44 * x92;
    double x94 = 1 - p_l[1] / p_h[1];
    double x95 = x42 * x94;
    double x96 = 1 - p_l[13] / p_h[13];
    double x97 = x46 * x96;
    double x98 = 1 - p_l[14] / p_h[14];
    double x99 = x48 * x98;
    double x100 = (p_h[14] * p_h[14]);
    double x101 = -(p_l[14] * p_l[14]) / x100 + 1;
    double x102 = u[14] * x101;
    double x103 = (p_h[15] * p_h[15]);
    double x104 = -(p_l[15] * p_l[15]) / x103 + 1;
    double x105 = u[15] * x104;
    double x106 = (p_h[16] * p_h[16]);
    double x107 = -(p_l[16] * p_l[16]) / x106 + 1;
    double x108 = u[16] * x107;
    double x109 = (p_h[17] * p_h[17]);
    double x110 = -(p_l[17] * p_l[17]) / x109 + 1;
    double x111 = u[17] * x110;
    double x112 = (p_h[18] * p_h[18]);
    double x113 = -(p_l[18] * p_l[18]) / x112 + 1;
    double x114 = u[18] * x113;
    double x115 = (p_h[19] * p_h[19]);
    double x116 = -(p_l[19] * p_l[19]) / x115 + 1;
    double x117 = u[19] * x116;
    double x118 = (p_h[1] * p_h[1]);
    double x119 = -(p_l[1] * p_l[1]) / x118 + 1;
    double x120 = u[1] * x119;
    double x121 = (p_h[20] * p_h[20]);
    double x122 = -(p_l[20] * p_l[20]) / x121 + 1;
    double x123 = u[20] * x122;
    double x124 = (p_h[21] * p_h[21]);
    double x125 = -(p_l[21] * p_l[21]) / x124 + 1;
    double x126 = u[21] * x125;
    double x127 = (p_h[22] * p_h[22]);
    double x128 = -(p_l[22] * p_l[22]) / x127 + 1;
    double x129 = u[22] * x128;
    double x130 = (p_h[23] * p_h[23]);
    double x131 = -(p_l[23] * p_l[23]) / x130 + 1;
    double x132 = u[23] * x131;
    double x133 = (p_h[24] * p_h[24]);
    double x134 = -(p_l[24] * p_l[24]) / x133 + 1;
    double x135 = u[24] * x134;
    double x136 = (p_h[2] * p_h[2]);
    double x137 = -(p_l[2] * p_l[2]) / x136 + 1;
    double x138 = u[2] * x137;
    double x139 = (p_h[3] * p_h[3]);
    double x140 = -(p_l[3] * p_l[3]) / x139 + 1;
    double x141 = u[3] * x140;
    double x142 = (p_h[4] * p_h[4]);
    double x143 = -(p_l[4] * p_l[4]) / x142 + 1;
    double x144 = u[4] * x143;
    double x145 = (p_h[5] * p_h[5]);
    double x146 = -(p_l[5] * p_l[5]) / x145 + 1;
    double x147 = u[5] * x146;
    double x148 = (p_h[6] * p_h[6]);
    double x149 = -(p_l[6] * p_l[6]) / x148 + 1;
    double x150 = u[6] * x149;
    double x151 = (p_h[7] * p_h[7]);
    double x152 = -(p_l[7] * p_l[7]) / x151 + 1;
    double x153 = u[7] * x152;
    double x154 = (p_h[8] * p_h[8]);
    double x155 = -(p_l[8] * p_l[8]) / x154 + 1;
    double x156 = u[8] * x155;
    double x157 = (p_h[9] * p_h[9]);
    double x158 = -(p_l[9] * p_l[9]) / x157 + 1;
    double x159 = u[9] * x158;
    double x160 = (p_h[0] * p_h[0]);
    double x161 = -(p_l[0] * p_l[0]) / x160 + 1;
    double x162 = u[0] * x161;
    double x163 = (p_h[10] * p_h[10]);
    double x164 = -(p_l[10] * p_l[10]) / x163 + 1;
    double x165 = u[10] * x164;
    double x166 = (p_h[11] * p_h[11]);
    double x167 = -(p_l[11] * p_l[11]) / x166 + 1;
    double x168 = u[11] * x167;
    double x169 = (p_h[12] * p_h[12]);
    double x170 = -(p_l[12] * p_l[12]) / x169 + 1;
    double x171 = u[12] * x170;
    double x172 = (p_h[13] * p_h[13]);
    double x173 = -(p_l[13] * p_l[13]) / x172 + 1;
    double x174 = u[13] * x173;
    double x175 = p_h[14] * x102;
    double x176 = p_h[15] * x105;
    double x177 = p_h[16] * x108;
    double x178 = p_h[1] * x120;
    double x179 = p_h[17] * x111;
    double x180 = p_h[18] * x114;
    double x181 = p_h[19] * x117;
    double x182 = p_h[20] * x123;
    double x183 = p_h[21] * x126;
    double x184 = p_h[22] * x129;
    double x185 = p_h[23] * x132;
    double x186 = p_h[24] * x135;
    double x187 = p_h[2] * x138;
    double x188 = p_h[3] * x141;
    double x189 = p_h[4] * x144;
    double x190 = p_h[5] * x147;
    double x191 = p_h[6] * x150;
    double x192 = p_h[0] * x162;
    double x193 = p_h[7] * x153;
    double x194 = p_h[8] * x156;
    double x195 = p_h[9] * x159;
    double x196 = p_h[10] * x165;
    double x197 = p_h[11] * x168;
    double x198 = p_h[12] * x171;
    double x199 = p_h[13] * x174;
    double x200 = (u[14] * u[14]);
    double x201 = p_h[14] * x101 * x200;
    double x202 = (u[15] * u[15]);
    double x203 = p_h[15] * x104 * x202;
    double x204 = (u[16] * u[16]);
    double x205 = p_h[16] * x107 * x204;
    double x206 = (u[17] * u[17]);
    double x207 = p_h[17] * x110 * x206;
    double x208 = (u[1] * u[1]);
    double x209 = p_h[1] * x119 * x208;
    double x210 = (u[18] * u[18]);
    double x211 = p_h[18] * x113 * x210;
    double x212 = (u[19] * u[19]);
    double x213 = p_h[19] * x116 * x212;
    double x214 = (u[20] * u[20]);
    double x215 = p_h[20] * x122 * x214;
    double x216 = (u[21] * u[21]);
    double x217 = p_h[21] * x125 * x216;
    double x218 = (u[22] * u[22]);
    double x219 = p_h[22] * x128 * x218;
    double x220 = (u[23] * u[23]);
    double x221 = p_h[23] * x131 * x220;
    double x222 = (u[24] * u[24]);
    double x223 = p_h[24] * x134 * x222;
    double x224 = (u[2] * u[2]);
    double x225 = p_h[2] * x137 * x224;
    double x226 = (u[3] * u[3]);
    double x227 = p_h[3] * x140 * x226;
    double x228 = (u[4] * u[4]);
    double x229 = p_h[4] * x143 * x228;
    double x230 = (u[5] * u[5]);
    double x231 = p_h[5] * x146 * x230;
    double x232 = (u[6] * u[6]);
    double x233 = p_h[6] * x149 * x232;
    double x234 = (u[7] * u[7]);
    double x235 = p_h[7] * x152 * x234;
    double x236 = (u[0] * u[0]);
    double x237 = p_h[0] * x161 * x236;
    double x238 = (u[8] * u[8]);
    double x239 = p_h[8] * x155 * x238;
    double x240 = (u[9] * u[9]);
    double x241 = p_h[9] * x158 * x240;
    double x242 = (u[10] * u[10]);
    double x243 = p_h[10] * x164 * x242;
    double x244 = (u[11] * u[11]);
    double x245 = p_h[11] * x167 * x244;
    double x246 = (u[12] * u[12]);
    double x247 = p_h[12] * x170 * x246;
    double x248 = (u[13] * u[13]);
    double x249 = p_h[13] * x173 * x248;
    double x250 = x100 * x200 * (x98 * x98);
    double x251 = (p_h[14] * p_h[14] * p_h[14]) * (u[14] * u[14] * u[14]) * (x98 * x98 * x98);
    double x252 = x103 * x202 * (x50 * x50);
    double x253 = (p_h[15] * p_h[15] * p_h[15]) * (u[15] * u[15] * u[15]) * (x50 * x50 * x50);
    double x254 = (p_h[1] * p_h[1] * p_h[1]) * (u[1] * u[1] * u[1]) * (x94 * x94 * x94);
    double x255 = x106 * x204 * (x52 * x52);
    double x256 = (p_h[16] * p_h[16] * p_h[16]) * (u[16] * u[16] * u[16]) * (x52 * x52 * x52);
    double x257 = x109 * x206 * (x54 * x54);
    double x258 = (p_h[17] * p_h[17] * p_h[17]) * (u[17] * u[17] * u[17]) * (x54 * x54 * x54);
    double x259 = x112 * x210 * (x56 * x56);
    double x260 = (p_h[18] * p_h[18] * p_h[18]) * (u[18] * u[18] * u[18]) * (x56 * x56 * x56);
    double x261 = x115 * x212 * (x58 * x58);
    double x262 = (p_h[19] * p_h[19] * p_h[19]) * (u[19] * u[19] * u[19]) * (x58 * x58 * x58);
    double x263 = x121 * x214 * (x60 * x60);
    double x264 = (p_h[20] * p_h[20] * p_h[20]) * (u[20] * u[20] * u[20]) * (x60 * x60 * x60);
    double x265 = x124 * x216 * (x62 * x62);
    double x266 = (p_h[21] * p_h[21] * p_h[21]) * (u[21] * u[21] * u[21]) * (x62 * x62 * x62);
    double x267 = x127 * x218 * (x64 * x64);
    double x268 = (p_h[22] * p_h[22] * p_h[22]) * (u[22] * u[22] * u[22]) * (x64 * x64 * x64);
    double x269 = x130 * x220 * (x68 * x68);
    double x270 = (p_h[23] * p_h[23] * p_h[23]) * (u[23] * u[23] * u[23]) * (x68 * x68 * x68);
    double x271 = x136 * x224 * (x66 * x66);
    double x272 = x133 * x222 * (x70 * x70);
    double x273 = (p_h[24] * p_h[24] * p_h[24]) * (u[24] * u[24] * u[24]) * (x70 * x70 * x70);
    double x274 = (p_h[2] * p_h[2] * p_h[2]) * (u[2] * u[2] * u[2]) * (x66 * x66 * x66);
    double x275 = x139 * x226 * (x74 * x74);
    double x276 = (p_h[3] * p_h[3] * p_h[3]) * (u[3] * u[3] * u[3]) * (x74 * x74 * x74);
    double x277 = x160 * x236 * (x72 * x72);
    double x278 = x142 * x228 * (x76 * x76);
    double x279 = (p_h[4] * p_h[4] * p_h[4]) * (u[4] * u[4] * u[4]) * (x76 * x76 * x76);
    double x280 = x145 * x230 * (x78 * x78);
    double x281 = (p_h[5] * p_h[5] * p_h[5]) * (u[5] * u[5] * u[5]) * (x78 * x78 * x78);
    double x282 = (p_h[0] * p_h[0] * p_h[0]) * (u[0] * u[0] * u[0]) * (x72 * x72 * x72);
    double x283 = x148 * x232 * (x80 * x80);
    double x284 = (p_h[6] * p_h[6] * p_h[6]) * (u[6] * u[6] * u[6]) * (x80 * x80 * x80);
    double x285 = x151 * x234 * (x82 * x82);
    double x286 = (p_h[7] * p_h[7] * p_h[7]) * (u[7] * u[7] * u[7]) * (x82 * x82 * x82);
    double x287 = x154 * x238 * (x84 * x84);
    double x288 = (p_h[8] * p_h[8] * p_h[8]) * (u[8] * u[8] * u[8]) * (x84 * x84 * x84);
    double x289 = x157 * x240 * (x86 * x86);
    double x290 = (p_h[9] * p_h[9] * p_h[9]) * (u[9] * u[9] * u[9]) * (x86 * x86 * x86);
    double x291 = x163 * x242 * (x88 * x88);
    double x292 = (p_h[10] * p_h[10] * p_h[10]) * (u[10] * u[10] * u[10]) * (x88 * x88 * x88);
    double x293 = x166 * x244 * (x90 * x90);
    double x294 = (p_h[11] * p_h[11] * p_h[11]) * (u[11] * u[11] * u[11]) * (x90 * x90 * x90);
    double x295 = x169 * x246 * (x92 * x92);
    double x296 = (p_h[12] * p_h[12] * p_h[12]) * (u[12] * u[12] * u[12]) * (x92 * x92 * x92);
    double x297 = x172 * x248 * (x96 * x96);
    double x298 = (p_h[13] * p_h[13] * p_h[13]) * (u[13] * u[13] * u[13]) * (x96 * x96 * x96);
    double x299 = x118 * x208 * (x94 * x94);
    Eigen::Matrix<double, 2, 1> f;
    f << a[0] * x1 + a[100] * x250 + a[101] * x251 + a[102] * x175 + a[103] * x201 + a[104] * x102 + a[105] * x3 + a[106] * x51 + a[107] * x252 + a[108] * x253 + a[109] * x176 + a[10] * x254 + a[110] * x203 + a[111] * x105 + a[112] * x5 + a[113] * x53 + a[114] * x255 + a[115] * x256 + a[116] * x177 + a[117] * x205 + a[118] * x108 + a[119] * x7 + a[11] * x178 + a[120] * x55 + a[121] * x257 + a[122] * x258 + a[123] * x179 + a[124] * x207 + a[125] * x111 + a[126] * x9 + a[127] * x57 + a[128] * x259 + a[129] * x260 + a[12] * x209 + a[130] * x180 + a[131] * x211 + a[132] * x114 + a[133] * x11 + a[134] * x59 + a[135] * x261 + a[136] * x262 + a[137] * x181 + a[138] * x213 + a[139] * x117 + a[13] * x120 + a[140] * x13 + a[141] * x61 + a[142] * x263 + a[143] * x264 + a[144] * x182 + a[145] * x215 + a[146] * x123 + a[147] * x15 + a[148] * x63 + a[149] * x265 + a[14] * x17 + a[150] * x266 + a[151] * x183 + a[152] * x217 + a[153] * x126 + a[154] * x19 + a[155] * x65 + a[156] * x267 + a[157] * x268 + a[158] * x184 + a[159] * x219 + a[15] * x67 + a[160] * x129 + a[161] * x21 + a[162] * x69 + a[163] * x269 + a[164] * x270 + a[165] * x185 + a[166] * x221 + a[167] * x132 + a[168] * x23 + a[169] * x71 + a[16] * x271 + a[170] * x272 + a[171] * x273 + a[172] * x186 + a[173] * x223 + a[174] * x135 + a[17] * x274 + a[18] * x187 + a[19] * x225 + a[1] * x73 + a[20] * x138 + a[21] * x25 + a[22] * x75 + a[23] * x275 + a[24] * x276 + a[25] * x188 + a[26] * x227 + a[27] * x141 + a[28] * x27 + a[29] * x77 + a[2] * x277 + a[30] * x278 + a[31] * x279 + a[32] * x189 + a[33] * x229 + a[34] * x144 + a[35] * x29 + a[36] * x79 + a[37] * x280 + a[38] * x281 + a[39] * x190 + a[3] * x282 + a[40] * x231 + a[41] * x147 + a[42] * x31 + a[43] * x81 + a[44] * x283 + a[45] * x284 + a[46] * x191 + a[47] * x233 + a[48] * x150 + a[49] * x33 + a[4] * x192 + a[50] * x83 + a[51] * x285 + a[52] * x286 + a[53] * x193 + a[54] * x235 + a[55] * x153 + a[56] * x35 + a[57] * x85 + a[58] * x287 + a[59] * x288 + a[5] * x237 + a[60] * x194 + a[61] * x239 + a[62] * x156 + a[63] * x37 + a[64] * x87 + a[65] * x289 + a[66] * x290 + a[67] * x195 + a[68] * x241 + a[69] * x159 + a[6] * x162 + a[70] * x39 + a[71] * x89 + a[72] * x291 + a[73] * x292 + a[74] * x196 + a[75] * x243 + a[76] * x165 + a[77] * x41 + a[78] * x91 + a[79] * x293 + a[7] * x43 + a[80] * x294 + a[81] * x197 + a[82] * x245 + a[83] * x168 + a[84] * x45 + a[85] * x93 + a[86] * x295 + a[87] * x296 + a[88] * x198 + a[89] * x247 + a[8] * x95 + a[90] * x171 + a[91] * x47 + a[92] * x97 + a[93] * x297 + a[94] * x298 + a[95] * x199 + a[96] * x249 + a[97] * x174 + a[98] * x49 + a[99] * x99 + a[9] * x299, b[0] * x1 + b[100] * x250 + b[101] * x251 + b[102] * x175 + b[103] * x201 + b[104] * x102 + b[105] * x3 + b[106] * x51 + b[107] * x252 + b[108] * x253 + b[109] * x176 + b[10] * x254 + b[110] * x203 + b[111] * x105 + b[112] * x5 + b[113] * x53 + b[114] * x255 + b[115] * x256 + b[116] * x177 + b[117] * x205 + b[118] * x108 + b[119] * x7 + b[11] * x178 + b[120] * x55 + b[121] * x257 + b[122] * x258 + b[123] * x179 + b[124] * x207 + b[125] * x111 + b[126] * x9 + b[127] * x57 + b[128] * x259 + b[129] * x260 + b[12] * x209 + b[130] * x180 + b[131] * x211 + b[132] * x114 + b[133] * x11 + b[134] * x59 + b[135] * x261 + b[136] * x262 + b[137] * x181 + b[138] * x213 + b[139] * x117 + b[13] * x120 + b[140] * x13 + b[141] * x61 + b[142] * x263 + b[143] * x264 + b[144] * x182 + b[145] * x215 + b[146] * x123 + b[147] * x15 + b[148] * x63 + b[149] * x265 + b[14] * x17 + b[150] * x266 + b[151] * x183 + b[152] * x217 + b[153] * x126 + b[154] * x19 + b[155] * x65 + b[156] * x267 + b[157] * x268 + b[158] * x184 + b[159] * x219 + b[15] * x67 + b[160] * x129 + b[161] * x21 + b[162] * x69 + b[163] * x269 + b[164] * x270 + b[165] * x185 + b[166] * x221 + b[167] * x132 + b[168] * x23 + b[169] * x71 + b[16] * x271 + b[170] * x272 + b[171] * x273 + b[172] * x186 + b[173] * x223 + b[174] * x135 + b[17] * x274 + b[18] * x187 + b[19] * x225 + b[1] * x73 + b[20] * x138 + b[21] * x25 + b[22] * x75 + b[23] * x275 + b[24] * x276 + b[25] * x188 + b[26] * x227 + b[27] * x141 + b[28] * x27 + b[29] * x77 + b[2] * x277 + b[30] * x278 + b[31] * x279 + b[32] * x189 + b[33] * x229 + b[34] * x144 + b[35] * x29 + b[36] * x79 + b[37] * x280 + b[38] * x281 + b[39] * x190 + b[3] * x282 + b[40] * x231 + b[41] * x147 + b[42] * x31 + b[43] * x81 + b[44] * x283 + b[45] * x284 + b[46] * x191 + b[47] * x233 + b[48] * x150 + b[49] * x33 + b[4] * x192 + b[50] * x83 + b[51] * x285 + b[52] * x286 + b[53] * x193 + b[54] * x235 + b[55] * x153 + b[56] * x35 + b[57] * x85 + b[58] * x287 + b[59] * x288 + b[5] * x237 + b[60] * x194 + b[61] * x239 + b[62] * x156 + b[63] * x37 + b[64] * x87 + b[65] * x289 + b[66] * x290 + b[67] * x195 + b[68] * x241 + b[69] * x159 + b[6] * x162 + b[70] * x39 + b[71] * x89 + b[72] * x291 + b[73] * x292 + b[74] * x196 + b[75] * x243 + b[76] * x165 + b[77] * x41 + b[78] * x91 + b[79] * x293 + b[7] * x43 + b[80] * x294 + b[81] * x197 + b[82] * x245 + b[83] * x168 + b[84] * x45 + b[85] * x93 + b[86] * x295 + b[87] * x296 + b[88] * x198 + b[89] * x247 + b[8] * x95 + b[90] * x171 + b[91] * x47 + b[92] * x97 + b[93] * x297 + b[94] * x298 + b[95] * x199 + b[96] * x249 + b[97] * x174 + b[98] * x49 + b[99] * x99 + b[9] * x299;
    return f;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[169] * u[24];
    double x1 = (p_l[24] * p_l[24]);
    double x2 = p_h[24] * u[24] / x1;
    double x3 = p_l[24] / p_h[24];
    double x4 = 2 * u[24];
    double x5 = a[172] * x4;
    double x6 = a[174] * x4;
    double x7 = (p_h[24] * p_h[24]);
    double x8 = 1.0 / x7;
    double x9 = p_l[24] * x8;
    double x10 = (u[24] * u[24]);
    double x11 = 2 * x10;
    double x12 = a[173] * x11;
    double x13 = 1 - x3;
    double x14 = p_h[24] * x11;
    double x15 = a[170] * x14;
    double x16 = (x13 * x13);
    double x17 = (u[24] * u[24] * u[24]);
    double x18 = 3 * x17 * x7;
    double x19 = a[171] * x18;
    double x20 = u[24] / p_l[24];
    double x21 = x1 * x8;
    double x22 = x1 / (p_h[24] * p_h[24] * p_h[24]);
    double x23 = 1 - x21;
    double x24 = u[24] * x23;
    double x25 = x10 * x23;
    double x26 = p_l[24] * x11 * x13;
    double x27 = 3 * p_h[24] * p_l[24] * x16 * x17;
    double x28 = (x13 * x13 * x13);
    double x29 = b[169] * u[24];
    double x30 = b[172] * x4;
    double x31 = b[174] * x4;
    double x32 = b[173] * x11;
    double x33 = b[170] * x14;
    double x34 = b[171] * x18;
    Eigen::Matrix<double, 2, 2> dF_dx_T;
    dF_dx_T << -a[168] * x2 - x0 - x12 * x3 - x13 * x15 - x16 * x19 - x3 * x5 - x6 * x9, a[168] * x20 + a[170] * x26 + a[171] * x27 + a[172] * x24 + a[173] * x25 + x0 * x13 + x0 * x3 + x12 * x21 + x15 * x16 + x19 * x28 + x21 * x5 + x22 * x6, -b[168] * x2 - x13 * x33 - x16 * x34 - x29 - x3 * x30 - x3 * x32 - x31 * x9, b[168] * x20 + b[170] * x26 + b[171] * x27 + b[172] * x24 + b[173] * x25 + x13 * x29 + x16 * x33 + x21 * x30 + x21 * x32 + x22 * x31 + x28 * x34 + x29 * x3;
    return dF_dx_T;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL1_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[162] * u[23];
    double x1 = (p_l[23] * p_l[23]);
    double x2 = p_h[23] * u[23] / x1;
    double x3 = p_l[23] / p_h[23];
    double x4 = 2 * u[23];
    double x5 = a[165] * x4;
    double x6 = a[167] * x4;
    double x7 = (p_h[23] * p_h[23]);
    double x8 = 1.0 / x7;
    double x9 = p_l[23] * x8;
    double x10 = (u[23] * u[23]);
    double x11 = 2 * x10;
    double x12 = a[166] * x11;
    double x13 = 1 - x3;
    double x14 = p_h[23] * x11;
    double x15 = a[163] * x14;
    double x16 = (x13 * x13);
    double x17 = (u[23] * u[23] * u[23]);
    double x18 = 3 * x17 * x7;
    double x19 = a[164] * x18;
    double x20 = u[23] / p_l[23];
    double x21 = x1 * x8;
    double x22 = x1 / (p_h[23] * p_h[23] * p_h[23]);
    double x23 = 1 - x21;
    double x24 = u[23] * x23;
    double x25 = x10 * x23;
    double x26 = p_l[23] * x11 * x13;
    double x27 = 3 * p_h[23] * p_l[23] * x16 * x17;
    double x28 = (x13 * x13 * x13);
    double x29 = b[162] * u[23];
    double x30 = b[165] * x4;
    double x31 = b[167] * x4;
    double x32 = b[166] * x11;
    double x33 = b[163] * x14;
    double x34 = b[164] * x18;

    Eigen::Matrix<double, 2, 2> dF_dx2_T;
    dF_dx2_T << -a[161] * x2 - x0 - x12 * x3 - x13 * x15 - x16 * x19 - x3 * x5 - x6 * x9, a[161] * x20 + a[163] * x26 + a[164] * x27 + a[165] * x24 + a[166] * x25 + x0 * x13 + x0 * x3 + x12 * x21 + x15 * x16 + x19 * x28 + x21 * x5 + x22 * x6, -b[161] * x2 - x13 * x33 - x16 * x34 - x29 - x3 * x30 - x3 * x32 - x31 * x9, b[161] * x20 + b[163] * x26 + b[164] * x27 + b[165] * x24 + b[166] * x25 + x13 * x29 + x16 * x33 + x21 * x30 + x21 * x32 + x22 * x31 + x28 * x34 + x29 * x3;
    return dF_dx2_T;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL2_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[155] * u[22];
    double x1 = (p_l[22] * p_l[22]);
    double x2 = p_h[22] * u[22] / x1;
    double x3 = p_l[22] / p_h[22];
    double x4 = 2 * u[22];
    double x5 = a[158] * x4;
    double x6 = a[160] * x4;
    double x7 = (p_h[22] * p_h[22]);
    double x8 = 1.0 / x7;
    double x9 = p_l[22] * x8;
    double x10 = (u[22] * u[22]);
    double x11 = 2 * x10;
    double x12 = a[159] * x11;
    double x13 = 1 - x3;
    double x14 = p_h[22] * x11;
    double x15 = a[156] * x14;
    double x16 = (x13 * x13);
    double x17 = (u[22] * u[22] * u[22]);
    double x18 = 3 * x17 * x7;
    double x19 = a[157] * x18;
    double x20 = u[22] / p_l[22];
    double x21 = x1 * x8;
    double x22 = x1 / (p_h[22] * p_h[22] * p_h[22]);
    double x23 = 1 - x21;
    double x24 = u[22] * x23;
    double x25 = x10 * x23;
    double x26 = p_l[22] * x11 * x13;
    double x27 = 3 * p_h[22] * p_l[22] * x16 * x17;
    double x28 = (x13 * x13 * x13);
    double x29 = b[155] * u[22];
    double x30 = b[158] * x4;
    double x31 = b[160] * x4;
    double x32 = b[159] * x11;
    double x33 = b[156] * x14;
    double x34 = b[157] * x18;

    Eigen::Matrix<double, 2, 2> dF_dx3_T;
    dF_dx3_T << -a[154] * x2 - x0 - x12 * x3 - x13 * x15 - x16 * x19 - x3 * x5 - x6 * x9, a[154] * x20 + a[156] * x26 + a[157] * x27 + a[158] * x24 + a[159] * x25 + x0 * x13 + x0 * x3 + x12 * x21 + x15 * x16 + x19 * x28 + x21 * x5 + x22 * x6, -b[154] * x2 - x13 * x33 - x16 * x34 - x29 - x3 * x30 - x3 * x32 - x31 * x9, b[154] * x20 + b[156] * x26 + b[157] * x27 + b[158] * x24 + b[159] * x25 + x13 * x29 + x16 * x33 + x21 * x30 + x21 * x32 + x22 * x31 + x28 * x34 + x29 * x3;
    return dF_dx3_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL3_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[148] * u[21];
    double x1 = (p_l[21] * p_l[21]);
    double x2 = p_h[21] * u[21] / x1;
    double x3 = p_l[21] / p_h[21];
    double x4 = 2 * u[21];
    double x5 = a[151] * x4;
    double x6 = a[153] * x4;
    double x7 = (p_h[21] * p_h[21]);
    double x8 = 1.0 / x7;
    double x9 = p_l[21] * x8;
    double x10 = (u[21] * u[21]);
    double x11 = 2 * x10;
    double x12 = a[152] * x11;
    double x13 = 1 - x3;
    double x14 = p_h[21] * x11;
    double x15 = a[149] * x14;
    double x16 = (x13 * x13);
    double x17 = (u[21] * u[21] * u[21]);
    double x18 = 3 * x17 * x7;
    double x19 = a[150] * x18;
    double x20 = u[21] / p_l[21];
    double x21 = x1 * x8;
    double x22 = x1 / (p_h[21] * p_h[21] * p_h[21]);
    double x23 = 1 - x21;
    double x24 = u[21] * x23;
    double x25 = x10 * x23;
    double x26 = p_l[21] * x11 * x13;
    double x27 = 3 * p_h[21] * p_l[21] * x16 * x17;
    double x28 = (x13 * x13 * x13);
    double x29 = b[148] * u[21];
    double x30 = b[151] * x4;
    double x31 = b[153] * x4;
    double x32 = b[152] * x11;
    double x33 = b[149] * x14;
    double x34 = b[150] * x18;
    Eigen::Matrix<double, 2, 2> dF_dx4_T;
    dF_dx4_T << -a[147] * x2 - x0 - x12 * x3 - x13 * x15 - x16 * x19 - x3 * x5 - x6 * x9, a[147] * x20 + a[149] * x26 + a[150] * x27 + a[151] * x24 + a[152] * x25 + x0 * x13 + x0 * x3 + x12 * x21 + x15 * x16 + x19 * x28 + x21 * x5 + x22 * x6, -b[147] * x2 - x13 * x33 - x16 * x34 - x29 - x3 * x30 - x3 * x32 - x31 * x9, b[147] * x20 + b[149] * x26 + b[150] * x27 + b[151] * x24 + b[152] * x25 + x13 * x29 + x16 * x33 + x21 * x30 + x21 * x32 + x22 * x31 + x28 * x34 + x29 * x3;
    return dF_dx4_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxL4_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = a[141] * u[20];
    double x1 = (p_l[20] * p_l[20]);
    double x2 = p_h[20] * u[20] / x1;
    double x3 = p_l[20] / p_h[20];
    double x4 = 2 * u[20];
    double x5 = a[144] * x4;
    double x6 = a[146] * x4;
    double x7 = (p_h[20] * p_h[20]);
    double x8 = 1.0 / x7;
    double x9 = p_l[20] * x8;
    double x10 = (u[20] * u[20]);
    double x11 = 2 * x10;
    double x12 = a[145] * x11;
    double x13 = 1 - x3;
    double x14 = p_h[20] * x11;
    double x15 = a[142] * x14;
    double x16 = (x13 * x13);
    double x17 = (u[20] * u[20] * u[20]);
    double x18 = 3 * x17 * x7;
    double x19 = a[143] * x18;
    double x20 = u[20] / p_l[20];
    double x21 = x1 * x8;
    double x22 = x1 / (p_h[20] * p_h[20] * p_h[20]);
    double x23 = 1 - x21;
    double x24 = u[20] * x23;
    double x25 = x10 * x23;
    double x26 = p_l[20] * x11 * x13;
    double x27 = 3 * p_h[20] * p_l[20] * x16 * x17;
    double x28 = (x13 * x13 * x13);
    double x29 = b[141] * u[20];
    double x30 = b[144] * x4;
    double x31 = b[146] * x4;
    double x32 = b[145] * x11;
    double x33 = b[142] * x14;
    double x34 = b[143] * x18;

    Eigen::Matrix<double, 2, 2> dF_dx5_T;
    dF_dx5_T << -a[140] * x2 - x0 - x12 * x3 - x13 * x15 - x16 * x19 - x3 * x5 - x6 * x9, a[140] * x20 + a[142] * x26 + a[143] * x27 + a[144] * x24 + a[145] * x25 + x0 * x13 + x0 * x3 + x12 * x21 + x15 * x16 + x19 * x28 + x21 * x5 + x22 * x6, -b[140] * x2 - x13 * x33 - x16 * x34 - x29 - x3 * x30 - x3 * x32 - x31 * x9, b[140] * x20 + b[142] * x26 + b[143] * x27 + b[144] * x24 + b[145] * x25 + x13 * x29 + x16 * x33 + x21 * x30 + x21 * x32 + x22 * x31 + x28 * x34 + x29 * x3;
    return dF_dx5_T;
}

Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[24] / p_l[24];
    double x1 = p_l[24] / p_h[24];
    double x2 = a[169] * u[24];
    double x3 = (p_h[24] * p_h[24]);
    double x4 = 1.0 / x3;
    double x5 = (p_l[24] * p_l[24]);
    double x6 = x4 * x5;
    double x7 = 2 * u[24];
    double x8 = a[172] * x7;
    double x9 = a[174] * x7;
    double x10 = x5 / (p_h[24] * p_h[24] * p_h[24]);
    double x11 = 1 - x1;
    double x12 = (u[24] * u[24]);
    double x13 = 2 * x12;
    double x14 = a[173] * x13;
    double x15 = 1 - x6;
    double x16 = u[24] * x15;
    double x17 = x12 * x15;
    double x18 = a[170] * x13;
    double x19 = x11 * x18;
    double x20 = (x11 * x11);
    double x21 = p_h[24] * x20;
    double x22 = 3 * (u[24] * u[24] * u[24]);
    double x23 = p_l[24] * x21 * x22;
    double x24 = (x11 * x11 * x11);
    double x25 = x22 * x3;
    double x26 = a[171] * x25;
    double x27 = p_h[24] * u[24] / x5;
    double x28 = p_l[24] * x4;
    double x29 = b[169] * u[24];
    double x30 = b[172] * x7;
    double x31 = b[174] * x7;
    double x32 = b[173] * x13;
    double x33 = b[170] * x13;
    double x34 = x11 * x33;
    double x35 = b[171] * x25;
    Eigen::Matrix<double, 2, 2> dF_dx_T;
    dF_dx_T << a[168] * x0 + a[171] * x23 + a[172] * x16 + a[173] * x17 + p_l[24] * x19 + x1 * x2 + x10 * x9 + x11 * x2 + x14 * x6 + x18 * x21 + x24 * x26 + x6 * x8, -a[168] * x27 - p_h[24] * x19 - x1 * x14 - x1 * x8 - x2 - x20 * x26 - x28 * x9, b[168] * x0 + b[171] * x23 + b[172] * x16 + b[173] * x17 + p_l[24] * x34 + x1 * x29 + x10 * x31 + x11 * x29 + x21 * x33 + x24 * x35 + x30 * x6 + x32 * x6, -b[168] * x27 - p_h[24] * x34 - x1 * x30 - x1 * x32 - x20 * x35 - x28 * x31 - x29;
    return dF_dx_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH1_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[23] / p_l[23];
    double x1 = p_l[23] / p_h[23];
    double x2 = a[162] * u[23];
    double x3 = (p_h[23] * p_h[23]);
    double x4 = 1.0 / x3;
    double x5 = (p_l[23] * p_l[23]);
    double x6 = x4 * x5;
    double x7 = 2 * u[23];
    double x8 = a[165] * x7;
    double x9 = a[167] * x7;
    double x10 = x5 / (p_h[23] * p_h[23] * p_h[23]);
    double x11 = 1 - x1;
    double x12 = (u[23] * u[23]);
    double x13 = 2 * x12;
    double x14 = a[166] * x13;
    double x15 = 1 - x6;
    double x16 = u[23] * x15;
    double x17 = x12 * x15;
    double x18 = a[163] * x13;
    double x19 = x11 * x18;
    double x20 = (x11 * x11);
    double x21 = p_h[23] * x20;
    double x22 = 3 * (u[23] * u[23] * u[23]);
    double x23 = p_l[23] * x21 * x22;
    double x24 = (x11 * x11 * x11);
    double x25 = x22 * x3;
    double x26 = a[164] * x25;
    double x27 = p_h[23] * u[23] / x5;
    double x28 = p_l[23] * x4;
    double x29 = b[162] * u[23];
    double x30 = b[165] * x7;
    double x31 = b[167] * x7;
    double x32 = b[166] * x13;
    double x33 = b[163] * x13;
    double x34 = x11 * x33;
    double x35 = b[164] * x25;

    Eigen::Matrix<double, 2, 2> dF_dx2_T;
    dF_dx2_T << a[161] * x0 + a[164] * x23 + a[165] * x16 + a[166] * x17 + p_l[23] * x19 + x1 * x2 + x10 * x9 + x11 * x2 + x14 * x6 + x18 * x21 + x24 * x26 + x6 * x8, -a[161] * x27 - p_h[23] * x19 - x1 * x14 - x1 * x8 - x2 - x20 * x26 - x28 * x9, b[161] * x0 + b[164] * x23 + b[165] * x16 + b[166] * x17 + p_l[23] * x34 + x1 * x29 + x10 * x31 + x11 * x29 + x21 * x33 + x24 * x35 + x30 * x6 + x32 * x6, -b[161] * x27 - p_h[23] * x34 - x1 * x30 - x1 * x32 - x20 * x35 - x28 * x31 - x29;
    return dF_dx2_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH2_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[22] / p_l[22];
    double x1 = p_l[22] / p_h[22];
    double x2 = a[155] * u[22];
    double x3 = (p_h[22] * p_h[22]);
    double x4 = 1.0 / x3;
    double x5 = (p_l[22] * p_l[22]);
    double x6 = x4 * x5;
    double x7 = 2 * u[22];
    double x8 = a[158] * x7;
    double x9 = a[160] * x7;
    double x10 = x5 / (p_h[22] * p_h[22] * p_h[22]);
    double x11 = 1 - x1;
    double x12 = (u[22] * u[22]);
    double x13 = 2 * x12;
    double x14 = a[159] * x13;
    double x15 = 1 - x6;
    double x16 = u[22] * x15;
    double x17 = x12 * x15;
    double x18 = a[156] * x13;
    double x19 = x11 * x18;
    double x20 = (x11 * x11);
    double x21 = p_h[22] * x20;
    double x22 = 3 * (u[22] * u[22] * u[22]);
    double x23 = p_l[22] * x21 * x22;
    double x24 = (x11 * x11 * x11);
    double x25 = x22 * x3;
    double x26 = a[157] * x25;
    double x27 = p_h[22] * u[22] / x5;
    double x28 = p_l[22] * x4;
    double x29 = b[155] * u[22];
    double x30 = b[158] * x7;
    double x31 = b[160] * x7;
    double x32 = b[159] * x13;
    double x33 = b[156] * x13;
    double x34 = x11 * x33;
    double x35 = b[157] * x25;
    Eigen::Matrix<double, 2, 2> dF_dx3_T;
    dF_dx3_T << a[154] * x0 + a[157] * x23 + a[158] * x16 + a[159] * x17 + p_l[22] * x19 + x1 * x2 + x10 * x9 + x11 * x2 + x14 * x6 + x18 * x21 + x24 * x26 + x6 * x8, -a[154] * x27 - p_h[22] * x19 - x1 * x14 - x1 * x8 - x2 - x20 * x26 - x28 * x9, b[154] * x0 + b[157] * x23 + b[158] * x16 + b[159] * x17 + p_l[22] * x34 + x1 * x29 + x10 * x31 + x11 * x29 + x21 * x33 + x24 * x35 + x30 * x6 + x32 * x6, -b[154] * x27 - p_h[22] * x34 - x1 * x30 - x1 * x32 - x20 * x35 - x28 * x31 - x29;
    return dF_dx3_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH3_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[21] / p_l[21];
    double x1 = p_l[21] / p_h[21];
    double x2 = a[148] * u[21];
    double x3 = (p_h[21] * p_h[21]);
    double x4 = 1.0 / x3;
    double x5 = (p_l[21] * p_l[21]);
    double x6 = x4 * x5;
    double x7 = 2 * u[21];
    double x8 = a[151] * x7;
    double x9 = a[153] * x7;
    double x10 = x5 / (p_h[21] * p_h[21] * p_h[21]);
    double x11 = 1 - x1;
    double x12 = (u[21] * u[21]);
    double x13 = 2 * x12;
    double x14 = a[152] * x13;
    double x15 = 1 - x6;
    double x16 = u[21] * x15;
    double x17 = x12 * x15;
    double x18 = a[149] * x13;
    double x19 = x11 * x18;
    double x20 = (x11 * x11);
    double x21 = p_h[21] * x20;
    double x22 = 3 * (u[21] * u[21] * u[21]);
    double x23 = p_l[21] * x21 * x22;
    double x24 = (x11 * x11 * x11);
    double x25 = x22 * x3;
    double x26 = a[150] * x25;
    double x27 = p_h[21] * u[21] / x5;
    double x28 = p_l[21] * x4;
    double x29 = b[148] * u[21];
    double x30 = b[151] * x7;
    double x31 = b[153] * x7;
    double x32 = b[152] * x13;
    double x33 = b[149] * x13;
    double x34 = x11 * x33;
    double x35 = b[150] * x25;

    Eigen::Matrix<double, 2, 2> dF_dx4_T;
    dF_dx4_T << a[147] * x0 + a[150] * x23 + a[151] * x16 + a[152] * x17 + p_l[21] * x19 + x1 * x2 + x10 * x9 + x11 * x2 + x14 * x6 + x18 * x21 + x24 * x26 + x6 * x8, -a[147] * x27 - p_h[21] * x19 - x1 * x14 - x1 * x8 - x2 - x20 * x26 - x28 * x9, b[147] * x0 + b[150] * x23 + b[151] * x16 + b[152] * x17 + p_l[21] * x34 + x1 * x29 + x10 * x31 + x11 * x29 + x21 * x33 + x24 * x35 + x30 * x6 + x32 * x6, -b[147] * x27 - p_h[21] * x34 - x1 * x30 - x1 * x32 - x20 * x35 - x28 * x31 - x29;
    return dF_dx4_T;
}
Eigen::Matrix<double, 2, 2> MPC::Update_dF_dxH4_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = u[20] / p_l[20];
    double x1 = p_l[20] / p_h[20];
    double x2 = a[141] * u[20];
    double x3 = (p_h[20] * p_h[20]);
    double x4 = 1.0 / x3;
    double x5 = (p_l[20] * p_l[20]);
    double x6 = x4 * x5;
    double x7 = 2 * u[20];
    double x8 = a[144] * x7;
    double x9 = a[146] * x7;
    double x10 = x5 / (p_h[20] * p_h[20] * p_h[20]);
    double x11 = 1 - x1;
    double x12 = (u[20] * u[20]);
    double x13 = 2 * x12;
    double x14 = a[145] * x13;
    double x15 = 1 - x6;
    double x16 = u[20] * x15;
    double x17 = x12 * x15;
    double x18 = a[142] * x13;
    double x19 = x11 * x18;
    double x20 = (x11 * x11);
    double x21 = p_h[20] * x20;
    double x22 = 3 * (u[20] * u[20] * u[20]);
    double x23 = p_l[20] * x21 * x22;
    double x24 = (x11 * x11 * x11);
    double x25 = x22 * x3;
    double x26 = a[143] * x25;
    double x27 = p_h[20] * u[20] / x5;
    double x28 = p_l[20] * x4;
    double x29 = b[141] * u[20];
    double x30 = b[144] * x7;
    double x31 = b[146] * x7;
    double x32 = b[145] * x13;
    double x33 = b[142] * x13;
    double x34 = x11 * x33;
    double x35 = b[143] * x25;
    Eigen::Matrix<double, 2, 2> dF_dx5_T;
    dF_dx5_T << a[140] * x0 + a[143] * x23 + a[144] * x16 + a[145] * x17 + p_l[20] * x19 + x1 * x2 + x10 * x9 + x11 * x2 + x14 * x6 + x18 * x21 + x24 * x26 + x6 * x8, -a[140] * x27 - p_h[20] * x19 - x1 * x14 - x1 * x8 - x2 - x20 * x26 - x28 * x9, b[140] * x0 + b[143] * x23 + b[144] * x16 + b[145] * x17 + p_l[20] * x34 + x1 * x29 + x10 * x31 + x11 * x29 + x21 * x33 + x24 * x35 + x30 * x6 + x32 * x6, -b[140] * x27 - p_h[20] * x34 - x1 * x30 - x1 * x32 - x20 * x35 - x28 * x31 - x29;
    return dF_dx5_T;
}

Eigen::Matrix<double, 2, 1> MPC::Update_dF_du_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[24] / p_l[24];
    double x1 = 1 - p_l[24] / p_h[24];
    double x2 = p_h[24] * x1;
    double x3 = (p_h[24] * p_h[24]);
    double x4 = -(p_l[24] * p_l[24]) / x3 + 1;
    double x5 = p_h[24] * x4;
    double x6 = 2 * u[24];
    double x7 = x5 * x6;
    double x8 = (x1 * x1) * x3 * x6;
    double x9 = 3 * (p_h[24] * p_h[24] * p_h[24]) * (u[24] * u[24]) * (x1 * x1 * x1);
    Eigen::Matrix<double, 2, 1> dF_du_T;
    dF_du_T << a[168] * x0 + a[169] * x2 + a[170] * x8 + a[171] * x9 + a[172] * x5 + a[173] * x7 + a[174] * x4, b[168] * x0 + b[169] * x2 + b[170] * x8 + b[171] * x9 + b[172] * x5 + b[173] * x7 + b[174] * x4;
    return dF_du_T;
}
Eigen::Matrix<double, 2, 1> MPC::Update_dF_du1_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[23] / p_l[23];
    double x1 = 1 - p_l[23] / p_h[23];
    double x2 = p_h[23] * x1;
    double x3 = (p_h[23] * p_h[23]);
    double x4 = -(p_l[23] * p_l[23]) / x3 + 1;
    double x5 = p_h[23] * x4;
    double x6 = 2 * u[23];
    double x7 = x5 * x6;
    double x8 = (x1 * x1) * x3 * x6;
    double x9 = 3 * (p_h[23] * p_h[23] * p_h[23]) * (u[23] * u[23]) * (x1 * x1 * x1);
    Eigen::Matrix<double, 2, 1> dF_du2_T;
    dF_du2_T << a[161] * x0 + a[162] * x2 + a[163] * x8 + a[164] * x9 + a[165] * x5 + a[166] * x7 + a[167] * x4, b[161] * x0 + b[162] * x2 + b[163] * x8 + b[164] * x9 + b[165] * x5 + b[166] * x7 + b[167] * x4;
    return dF_du2_T;
}

Eigen::Matrix<double, 2, 1> MPC::Update_dF_du2_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[22] / p_l[22];
    double x1 = 1 - p_l[22] / p_h[22];
    double x2 = p_h[22] * x1;
    double x3 = (p_h[22] * p_h[22]);
    double x4 = -(p_l[22] * p_l[22]) / x3 + 1;
    double x5 = p_h[22] * x4;
    double x6 = 2 * u[22];
    double x7 = x5 * x6;
    double x8 = (x1 * x1) * x3 * x6;
    double x9 = 3 * (p_h[22] * p_h[22] * p_h[22]) * (u[22] * u[22]) * (x1 * x1 * x1);
    Eigen::Matrix<double, 2, 1> dF_du3_T;
    dF_du3_T << a[154] * x0 + a[155] * x2 + a[156] * x8 + a[157] * x9 + a[158] * x5 + a[159] * x7 + a[160] * x4, b[154] * x0 + b[155] * x2 + b[156] * x8 + b[157] * x9 + b[158] * x5 + b[159] * x7 + b[160] * x4;
    return dF_du3_T;
}

Eigen::Matrix<double, 2, 1> MPC::Update_dF_du3_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[21] / p_l[21];
    double x1 = 1 - p_l[21] / p_h[21];
    double x2 = p_h[21] * x1;
    double x3 = (p_h[21] * p_h[21]);
    double x4 = -(p_l[21] * p_l[21]) / x3 + 1;
    double x5 = p_h[21] * x4;
    double x6 = 2 * u[21];
    double x7 = x5 * x6;
    double x8 = (x1 * x1) * x3 * x6;
    double x9 = 3 * (p_h[21] * p_h[21] * p_h[21]) * (u[21] * u[21]) * (x1 * x1 * x1);
    Eigen::Matrix<double, 2, 1> dF_du4_T;
    dF_du4_T << a[147] * x0 + a[148] * x2 + a[149] * x8 + a[150] * x9 + a[151] * x5 + a[152] * x7 + a[153] * x4, b[147] * x0 + b[148] * x2 + b[149] * x8 + b[150] * x9 + b[151] * x5 + b[152] * x7 + b[153] * x4;
    return dF_du4_T;
}
Eigen::Matrix<double, 2, 1> MPC::Update_dF_du4_T(const double *p_h, const double *p_l, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b)
{
    double x0 = p_h[20] / p_l[20];
    double x1 = 1 - p_l[20] / p_h[20];
    double x2 = p_h[20] * x1;
    double x3 = (p_h[20] * p_h[20]);
    double x4 = -(p_l[20] * p_l[20]) / x3 + 1;
    double x5 = p_h[20] * x4;
    double x6 = 2 * u[20];
    double x7 = x5 * x6;
    double x8 = (x1 * x1) * x3 * x6;
    double x9 = 3 * (p_h[20] * p_h[20] * p_h[20]) * (u[20] * u[20]) * (x1 * x1 * x1);
    Eigen::Matrix<double, 2, 1> dF_du5_T;
    dF_du5_T << a[140] * x0 + a[141] * x2 + a[142] * x8 + a[143] * x9 + a[144] * x5 + a[145] * x7 + a[146] * x4, b[140] * x0 + b[141] * x2 + b[142] * x8 + b[143] * x9 + b[144] * x5 + b[145] * x7 + b[146] * x4;
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
        std::array<double, 19>{this->cur_F.coeff(0, 0), this->cur_F.coeff(1, 0), this->cur_dF.coeff(0, 0), this->cur_dF.coeff(1, 0), this->u_n, this->u_n1, this->u_n2, this->u_n3, this->u_n4, this->x_n1.coeff(1, 0), this->x_n2.coeff(1, 0), this->x_n3.coeff(1, 0), this->x_n4.coeff(1, 0), this->x_n5.coeff(1, 0),this->y_des1,this->y_des2,this->y_des3,this->y_des4,this->y_des5});
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
