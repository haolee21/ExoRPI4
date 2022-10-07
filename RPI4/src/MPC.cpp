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
    this->osqp_settings->eps_abs = 0.000000001;
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

int MPC::DutyCalculate(bool increase_pre, std::array<float, MPC_TIME_HORIZON> y_des, double scale)
{

    this->y_des1 = y_des[0];
    this->y_des2 = y_des[1];
    this->y_des3 = y_des[2];
    this->y_des4 = y_des[3];
    this->y_des5 = y_des[4];
    this->y_des6 = y_des[5];
    this->y_des7 = y_des[6];
    this->y_des8 = y_des[7];
    this->y_des9 = y_des[8];



    Eigen::Matrix<double, 2, 1> f_p1, f_p2, f_p3, f_p4, f_p5, f_p6, f_p7, f_p8, f_p9;
    Eigen::Matrix<double, 2, 1> df_dun_p1_T, df_dun_p2_T, df_dun_p3_T, df_dun_p4_T, df_dun_p5_T, df_dun_p6_T, df_dun_p7_T, df_dun_p8_T, df_dun_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun1_p2_T, df_dun1_p3_T, df_dun1_p4_T, df_dun1_p5_T, df_dun1_p6_T, df_dun1_p7_T, df_dun1_p8_T, df_dun1_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun2_p3_T, df_dun2_p4_T, df_dun2_p5_T, df_dun2_p6_T, df_dun2_p7_T, df_dun2_p8_T, df_dun2_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun3_p4_T, df_dun3_p5_T, df_dun3_p6_T, df_dun3_p7_T, df_dun3_p8_T, df_dun3_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun4_p5_T, df_dun4_p6_T, df_dun4_p7_T, df_dun4_p8_T, df_dun4_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun5_p6_T, df_dun5_p7_T, df_dun5_p8_T, df_dun5_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun6_p7_T, df_dun6_p8_T, df_dun6_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun7_p8_T, df_dun7_p9_T;
    Eigen::Matrix<double, 2, 1> df_dun8_p9_T;

    Eigen::Matrix<double, 2, 2> df_dxn_p2_T, df_dxn_p3_T, df_dxn_p4_T, df_dxn_p5_T, df_dxn_p6_T, df_dxn_p7_T, df_dxn_p8_T, df_dxn_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn1_p3_T, df_dxn1_p4_T, df_dxn1_p5_T, df_dxn1_p6_T, df_dxn1_p7_T, df_dxn1_p8_T, df_dxn1_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn2_p4_T, df_dxn2_p5_T, df_dxn2_p6_T, df_dxn2_p7_T, df_dxn2_p8_T, df_dxn2_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn3_p5_T, df_dxn3_p6_T, df_dxn3_p7_T, df_dxn3_p8_T, df_dxn3_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn4_p6_T, df_dxn4_p7_T, df_dxn4_p8_T, df_dxn4_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn5_p7_T, df_dxn5_p8_T, df_dxn5_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn6_p8_T, df_dxn6_p9_T;
    Eigen::Matrix<double, 2, 2> df_dxn7_p9_T;

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
    }
    else
    {
        p_h = this->p_set_his.begin();
        p_l = this->p_tank_his.begin();
        cur_a = &this->al;
        cur_b = &this->bl;
    }

    
 

    f_p1 = scale * this->UpdateF(p_h, p_l, this->u_his.begin() + 1, *cur_a, *cur_b) + F_offset;
    f_p2 = scale * this->UpdateF(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b) + F_offset;
    f_p3 = scale * this->UpdateF(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b) + F_offset;
    f_p4 = scale * this->UpdateF(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b) + F_offset;
    f_p5 = scale * this->UpdateF(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b) + F_offset;
    f_p6 = scale * this->UpdateF(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b) + F_offset;
    f_p7 = scale * this->UpdateF(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b) + F_offset;
    f_p8 = scale * this->UpdateF(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b) + F_offset;
    f_p9 = scale * this->UpdateF(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b) + F_offset;

    df_dun_p1_T = scale * this->Update_dF_du_T(p_h, p_l, this->u_his.begin() + 1, *cur_a, *cur_b);
    df_dun_p2_T = scale * this->Update_dF_du_T(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun_p3_T = scale * this->Update_dF_du_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun_p4_T = scale * this->Update_dF_du_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun_p5_T = scale * this->Update_dF_du_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun_p6_T = scale * this->Update_dF_du_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun_p7_T = scale * this->Update_dF_du_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun_p8_T = scale * this->Update_dF_du_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun_p9_T = scale * this->Update_dF_du_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun1_p2_T = scale * this->Update_dF_du1_T(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun1_p3_T = scale * this->Update_dF_du1_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun1_p4_T = scale * this->Update_dF_du1_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun1_p5_T = scale * this->Update_dF_du1_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun1_p6_T = scale * this->Update_dF_du1_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun1_p7_T = scale * this->Update_dF_du1_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun1_p8_T = scale * this->Update_dF_du1_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun1_p9_T = scale * this->Update_dF_du1_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun2_p3_T = scale * this->Update_dF_du2_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun2_p4_T = scale * this->Update_dF_du2_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun2_p5_T = scale * this->Update_dF_du2_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun2_p6_T = scale * this->Update_dF_du2_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun2_p7_T = scale * this->Update_dF_du2_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun2_p8_T = scale * this->Update_dF_du2_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun2_p9_T = scale * this->Update_dF_du2_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun3_p4_T = scale * this->Update_dF_du3_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun3_p5_T = scale * this->Update_dF_du3_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun3_p6_T = scale * this->Update_dF_du3_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun3_p7_T = scale * this->Update_dF_du3_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun3_p8_T = scale * this->Update_dF_du3_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun3_p9_T = scale * this->Update_dF_du3_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun4_p5_T = scale * this->Update_dF_du4_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun4_p6_T = scale * this->Update_dF_du4_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun4_p7_T = scale * this->Update_dF_du4_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun4_p8_T = scale * this->Update_dF_du4_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun4_p9_T = scale * this->Update_dF_du4_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun5_p6_T = scale * this->Update_dF_du5_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun5_p7_T = scale * this->Update_dF_du5_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun5_p8_T = scale * this->Update_dF_du5_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun5_p9_T = scale * this->Update_dF_du5_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun6_p7_T = scale * this->Update_dF_du6_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun6_p8_T = scale * this->Update_dF_du6_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun6_p9_T = scale * this->Update_dF_du6_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun7_p8_T = scale * this->Update_dF_du7_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dun7_p9_T = scale * this->Update_dF_du7_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dun8_p9_T = scale * this->Update_dF_du8_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn_p2_T = scale * this->Update_dF_dxH_T(p_h + 1, p_l + 1, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dxn_p3_T = scale * this->Update_dF_dxH_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn_p4_T = scale * this->Update_dF_dxH_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn_p5_T = scale * this->Update_dF_dxH_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn_p6_T = scale * this->Update_dF_dxH_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn_p7_T = scale * this->Update_dF_dxH_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn_p8_T = scale * this->Update_dF_dxH_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn_p9_T = scale * this->Update_dF_dxH_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn1_p3_T = scale * this->Update_dF_dxH1_T(p_h + 2, p_l + 2, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn1_p4_T = scale * this->Update_dF_dxH1_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn1_p5_T = scale * this->Update_dF_dxH1_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn1_p6_T = scale * this->Update_dF_dxH1_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn1_p7_T = scale * this->Update_dF_dxH1_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn1_p8_T = scale * this->Update_dF_dxH1_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn1_p9_T = scale * this->Update_dF_dxH1_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn2_p4_T = scale * this->Update_dF_dxH2_T(p_h + 3, p_l + 3, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn2_p5_T = scale * this->Update_dF_dxH2_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn2_p6_T = scale * this->Update_dF_dxH2_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn2_p7_T = scale * this->Update_dF_dxH2_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn2_p8_T = scale * this->Update_dF_dxH2_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn2_p9_T = scale * this->Update_dF_dxH2_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn3_p5_T = scale * this->Update_dF_dxH3_T(p_h + 4, p_l + 4, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn3_p6_T = scale * this->Update_dF_dxH3_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn3_p7_T = scale * this->Update_dF_dxH3_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn3_p8_T = scale * this->Update_dF_dxH3_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn3_p9_T = scale * this->Update_dF_dxH3_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn4_p6_T = scale * this->Update_dF_dxH4_T(p_h + 5, p_l + 5, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn4_p7_T = scale * this->Update_dF_dxH4_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn4_p8_T = scale * this->Update_dF_dxH4_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn4_p9_T = scale * this->Update_dF_dxH4_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn5_p7_T = scale * this->Update_dF_dxH5_T(p_h + 6, p_l + 6, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn5_p8_T = scale * this->Update_dF_dxH5_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn5_p9_T = scale * this->Update_dF_dxH5_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn6_p8_T = scale * this->Update_dF_dxH6_T(p_h + 7, p_l + 7, this->u_his.begin() + 8, *cur_a, *cur_b);
    df_dxn6_p9_T = scale * this->Update_dF_dxH6_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    df_dxn7_p9_T = scale * this->Update_dF_dxH7_T(p_h + 8, p_l + 8, this->u_his.begin() + 9, *cur_a, *cur_b);

    Eigen::Matrix<double, 2, 1> x_bar;
    Eigen::Matrix<double, 2, 1> zero_col(0, 0);

    x_bar << this->p_tank_his[MPC_DELAY], this->p_set_his[MPC_DELAY];

    Eigen::Matrix<double, 2, 1> A1 = f_p1 - df_dun_p1_T * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B1;
    B1 << df_dun_p1_T, zero_col, zero_col, zero_col, zero_col, zero_col, zero_col, zero_col, zero_col;

    Eigen::Matrix<double, 2, 1> A2 = f_p2 + df_dxn_p2_T * A1 - (df_dun_p2_T + df_dun1_p2_T) * MPC::kUBar - df_dxn_p2_T * x_bar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B2;
    B2 << df_dun1_p2_T, df_dun_p2_T, zero_col, zero_col, zero_col, zero_col, zero_col, zero_col, zero_col;
    B2 += df_dxn_p2_T * B1;

    Eigen::Matrix<double, 2, 1> A3 = f_p3 + df_dxn_p3_T * (A2 - x_bar) + df_dxn1_p3_T * (A1 - x_bar) - (df_dun_p3_T + df_dun1_p3_T + df_dun2_p3_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B3;
    B3 << df_dun2_p3_T, df_dun1_p3_T, df_dun_p3_T, zero_col, zero_col, zero_col, zero_col, zero_col, zero_col;
    B3 += df_dxn_p3_T * B2 + df_dxn1_p3_T * B1;

    Eigen::Matrix<double, 2, 1> A4 = f_p4 + df_dxn_p4_T * (A3 - x_bar) + df_dxn1_p4_T * (A2 - x_bar) + df_dxn2_p4_T * (A1 - x_bar) - (df_dun_p4_T + df_dun1_p4_T + df_dun2_p4_T + df_dun3_p4_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B4;
    B4 << df_dun3_p4_T, df_dun2_p4_T, df_dun1_p4_T, df_dun_p4_T, zero_col, zero_col, zero_col, zero_col, zero_col;
    B4 += df_dxn_p4_T * B3 + df_dxn1_p4_T * B2 + df_dxn2_p4_T * B1;

    Eigen::Matrix<double, 2, 1> A5 = f_p5 + df_dxn_p5_T * (A4 - x_bar) + df_dxn1_p5_T * (A3 - x_bar) + df_dxn2_p5_T * (A2 - x_bar) + df_dxn3_p5_T * (A1 - x_bar) - (df_dun_p5_T + df_dun1_p5_T + df_dun2_p5_T + df_dun3_p5_T + df_dun4_p5_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B5;
    B5 << df_dun4_p5_T, df_dun3_p5_T, df_dun2_p5_T, df_dun1_p5_T, df_dun_p5_T, zero_col, zero_col, zero_col, zero_col;
    B5 += df_dxn_p5_T * B4 + df_dxn1_p5_T * B3 + df_dxn2_p5_T * B2 + df_dxn3_p5_T * B1;

    Eigen::Matrix<double, 2, 1> A6 = f_p6 + df_dxn_p6_T * (A5 - x_bar) + df_dxn1_p6_T * (A4 - x_bar) + df_dxn2_p6_T * (A3 - x_bar) + df_dxn3_p6_T * (A2 - x_bar) + df_dxn4_p6_T * (A1 - x_bar) - (df_dun_p6_T + df_dun1_p6_T + df_dun2_p6_T + df_dun3_p6_T + df_dun4_p6_T + df_dun5_p6_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B6;
    B6 << df_dun5_p6_T, df_dun4_p6_T, df_dun3_p6_T, df_dun2_p6_T, df_dun1_p6_T, df_dun_p6_T, zero_col, zero_col, zero_col;
    B6 += df_dxn_p6_T * B5 + df_dxn1_p6_T * B4 + df_dxn2_p6_T * B3 + df_dxn3_p6_T * B2 + df_dxn4_p6_T * B1;

    Eigen::Matrix<double, 2, 1> A7 = f_p7 + df_dxn_p7_T * (A6 - x_bar) + df_dxn1_p7_T * (A5 - x_bar) + df_dxn2_p7_T * (A4 - x_bar) + df_dxn3_p7_T * (A3 - x_bar) + df_dxn4_p7_T * (A2 - x_bar) + df_dxn5_p7_T * (A1 - x_bar) - (df_dun_p7_T + df_dun1_p7_T + df_dun2_p7_T + df_dun3_p7_T + df_dun4_p7_T + df_dun5_p7_T + df_dun6_p7_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B7;
    B7 << df_dun6_p7_T, df_dun5_p7_T, df_dun4_p7_T, df_dun3_p7_T, df_dun2_p7_T, df_dun1_p7_T, df_dun_p7_T, zero_col, zero_col;
    B7 += df_dxn_p7_T * B6 + df_dxn1_p7_T * B5 + df_dxn2_p7_T * B4 + df_dxn3_p7_T * B3 + df_dxn4_p7_T * B2 + df_dxn5_p7_T * B1;

    Eigen::Matrix<double, 2, 1> A8 = f_p8 + df_dxn_p8_T * (A7 - x_bar) + df_dxn1_p8_T * (A6 - x_bar) + df_dxn2_p8_T * (A5 - x_bar) + df_dxn3_p8_T * (A4 - x_bar) + df_dxn4_p8_T * (A3 - x_bar) + df_dxn5_p8_T * (A2 - x_bar) + df_dxn6_p8_T * (A1 - x_bar) - (df_dun_p8_T + df_dun1_p8_T + df_dun2_p8_T + df_dun3_p8_T + df_dun4_p8_T + df_dun5_p8_T + df_dun6_p8_T + df_dun7_p8_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B8;
    B8 << df_dun7_p8_T, df_dun6_p8_T, df_dun5_p8_T, df_dun4_p8_T, df_dun3_p8_T, df_dun2_p8_T, df_dun1_p8_T, df_dun_p8_T, zero_col;
    B8 += df_dxn_p8_T * B7 + df_dxn1_p8_T * B6 + df_dxn2_p8_T * B5 + df_dxn3_p8_T * B4 + df_dxn4_p8_T * B3 + df_dxn5_p8_T * B2 + df_dxn6_p8_T * B1;

    Eigen::Matrix<double, 2, 1> A9 = f_p9 + df_dxn_p9_T * (A8 - x_bar) + df_dxn1_p9_T * (A7 - x_bar) + df_dxn2_p9_T * (A6 - x_bar) + df_dxn3_p9_T * (A5 - x_bar) + df_dxn4_p9_T * (A4 - x_bar) + df_dxn5_p9_T * (A3 - x_bar) + df_dxn6_p9_T * (A2 - x_bar) + df_dxn7_p9_T * (A1 - x_bar) - (df_dun_p9_T + df_dun1_p9_T + df_dun2_p9_T + df_dun3_p9_T + df_dun4_p9_T + df_dun5_p9_T + df_dun6_p9_T + df_dun7_p9_T + df_dun8_p9_T) * MPC::kUBar;
    Eigen::Matrix<double, 2, MPC_TIME_HORIZON> B9;
    B9 << df_dun8_p9_T, df_dun7_p9_T, df_dun6_p9_T, df_dun5_p9_T, df_dun4_p9_T, df_dun3_p9_T, df_dun2_p9_T, df_dun1_p9_T, df_dun_p9_T;
    B9 += df_dxn_p9_T * B8 + df_dxn1_p9_T * B7 + df_dxn2_p9_T * B6 + df_dxn3_p9_T * B5 + df_dxn4_p9_T * B4 + df_dxn5_p9_T * B3 + df_dxn6_p9_T * B2 + df_dxn7_p9_T * B1;

    // combine all matrices
    Eigen::Matrix<double, 1, 2> H(0, 1);
    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> A_all;
    Eigen::Matrix<double, MPC_TIME_HORIZON, MPC_TIME_HORIZON> B_all;
    A_all << H * A1, H * A2, H * A3, H * A4, H * A5, H * A6, H * A7, H * A8, H * A9;
    B_all << H * B1, H * B2, H * B3, H * B4, H * B5, H * B6, H * B7, H * B8, H * B9;
    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> y_des_vec;
    y_des_vec << y_des[0], y_des[1], y_des[2], y_des[3], y_des[4], y_des[5], y_des[6], y_des[7], y_des[8];
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
                                                                  P_mat.coeff(0, 4), P_mat.coeff(1, 4), P_mat.coeff(2, 4), P_mat.coeff(3, 4), P_mat.coeff(4, 4),
                                                                  P_mat.coeff(0, 5), P_mat.coeff(1, 5), P_mat.coeff(2, 5), P_mat.coeff(3, 5), P_mat.coeff(4, 5), P_mat.coeff(5, 5),
                                                                  P_mat.coeff(0, 6), P_mat.coeff(1, 6), P_mat.coeff(2, 6), P_mat.coeff(3, 6), P_mat.coeff(4, 6), P_mat.coeff(5, 6), P_mat.coeff(6, 6),
                                                                  P_mat.coeff(0, 7), P_mat.coeff(1, 7), P_mat.coeff(2, 7), P_mat.coeff(3, 7), P_mat.coeff(4, 7), P_mat.coeff(5, 7), P_mat.coeff(6, 7), P_mat.coeff(7, 7),
                                                                  P_mat.coeff(0, 8), P_mat.coeff(1, 8), P_mat.coeff(2, 8), P_mat.coeff(3, 8), P_mat.coeff(4, 8), P_mat.coeff(5, 8), P_mat.coeff(6, 8), P_mat.coeff(7, 8), P_mat.coeff(8, 8)};
    c_int P_i[(MPC_TIME_HORIZON + 1) * MPC_TIME_HORIZON / 2] = {0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 8};
    c_int P_p[MPC_TIME_HORIZON + 1] = {0, 1, 3, 6, 10, 15, 21, 28, 36, 45};
    c_int P_nnz = (MPC_TIME_HORIZON + 1) * MPC_TIME_HORIZON / 2;
    c_float q[MPC_TIME_HORIZON] = {q_mat.coeff(0, 0), q_mat.coeff(0, 1), q_mat.coeff(0, 2), q_mat.coeff(0, 3), q_mat.coeff(0, 4), q_mat.coeff(0, 5), q_mat.coeff(0, 6), q_mat.coeff(0, 7), q_mat.coeff(0, 8)};

    c_int A_nnz = MPC_TIME_HORIZON;
    c_float A_x[MPC_TIME_HORIZON] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    c_int A_i[MPC_TIME_HORIZON] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    c_int A_p[MPC_TIME_HORIZON + 1] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    c_float l[MPC_TIME_HORIZON] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    c_float u[MPC_TIME_HORIZON] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
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

    if (exit_flag == 1)
    {
        std::cout << "osqp success" << std::endl;
    }

    this->u_n = *(this->work->solution->x);
    this->u_n1 = *(this->work->solution->x + 1);
    this->u_n2 = *(this->work->solution->x + 2);
    this->u_n3 = *(this->work->solution->x + 3);
    this->u_n4 = *(this->work->solution->x + 4);
    this->u_n5 = *(this->work->solution->x + 5);
    this->u_n6 = *(this->work->solution->x + 6);
    this->u_n7 = *(this->work->solution->x + 7);
    this->u_n8 = *(this->work->solution->x + 8);

    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> u_vec;
    u_vec << this->u_n, this->u_n1, this->u_n2, this->u_n3, this->u_n4, this->u_n5, this->u_n6, this->u_n7, this->u_n8;
    // std::cout<<"u vec: "<<u_vec<<std::endl;

    this->x_n1 = A1 + B1 * u_vec;
    this->x_n2 = A2 + B2 * u_vec;
    this->x_n3 = A3 + B3 * u_vec;
    this->x_n4 = A4 + B4 * u_vec;
    this->x_n5 = A5 + B5 * u_vec;
    this->x_n6 = A6 + B6 * u_vec;
    this->x_n7 = A7 + B7 * u_vec;
    this->x_n8 = A8 + B8 * u_vec;
    this->x_n9 = A9 + B9 * u_vec;

    // std::cout<<"u_n: "<<this->u_n<<", u_1: "<<this->u_n1<<", u_2: "<<this->u_n2<<", u_3: "<<this->u_n3<<", u_4: "<<this->u_n4<<std::endl;
    // std::cout<<"x_n1: "<<this->x_n1.coeff(1,0)<<", x_n2: "<<this->x_n2.coeff(1,0)<<", x_n3: "<<this->x_n3.coeff(1,0)<<std::endl;

    // auto res_err = u_vec.transpose()*P_mat*u_vec+2*q_mat*u_vec;
    // std::cout<<"residual: "<<res_err*2<<std::endl;
    // auto err = y_des_vec-A_all-B_all*u_vec;
    // auto res_err2 = err.norm();
    // std::cout<<"real residual: "<<res_err2<<std::endl;

    this->cur_dF = scale * this->UpdateF(p_h, p_l, this->u_his.begin(), *cur_a, *cur_b);
    return (int)(this->u_n * 100 + 0.5);
}

int MPC::GetPreControl(const double &p_des, const double &ps, const double &pt, double scale)
{

    double p_diff = (p_des - ps); // we scaled the p_diff with the assumption that pressure will have the momentum to go

    if (std::abs(p_diff) > 320) // 640 is 2 psi
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
        std::array<float, MPC_TIME_HORIZON> y_des{p_des_scale, p_des_scale, p_des_scale, p_des_scale, p_des_scale, p_des_scale, p_des_scale, p_des_scale, p_des_scale};
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

        if (ideal_duty <= 15)
        {
            return 0;
        }
        // else if (ideal_duty < 15 && ideal_duty > 10)
        // {
        //     // return 0;
        //     return 15;
        // }
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
        std::array<double, 31>{this->cur_F.coeff(0, 0), this->cur_F.coeff(1, 0), this->cur_dF.coeff(0, 0), this->cur_dF.coeff(1, 0),
         this->u_n, this->u_n1, this->u_n2, this->u_n3, this->u_n4,this->u_n5,this->u_n6,this->u_n7,this->u_n8,
          this->x_n1.coeff(1, 0), this->x_n2.coeff(1, 0), this->x_n3.coeff(1, 0), this->x_n4.coeff(1, 0), this->x_n5.coeff(1, 0), this->x_n6.coeff(1,0),this->x_n7.coeff(1,0),this->x_n8.coeff(1,0),this->x_n9.coeff(1,0),
          this->y_des1, this->y_des2, this->y_des3, this->y_des4, this->y_des5,this->y_des6,this->y_des7,this->y_des8,this->y_des9});
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
