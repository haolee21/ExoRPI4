#include <cmath>
#include <iostream>

#include "MPC.hpp"
// #include "MPC_param.hpp"

using namespace std;

// const double MPC::kArea =  0.31*645.16f;  //unit: mm^2

MPC::MPC(ExoConfig::MPC_Params _mpc_params,std::string file_name)
    : mpc_params(_mpc_params),ah(_mpc_params.ch[0]), bh(_mpc_params.ch[1]), al(_mpc_params.cl[0]), bl(_mpc_params.cl[1]),
      mpc_rec(file_name, MPC_HEAD),
      mpc_model_rec(file_name+std::string("_model"),MPC_MODEL_HEAD)

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
    this->cur_u=0.15;
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

double MPC::GetMpcCalibLen(){
    return this->mpc_params.cali_chamber_len;
}
void MPC::UpdateParam(ExoConfig::MPC_Params new_params){
    this->ah = new_params.ch[0];
    this->bh = new_params.ch[1];
    this->al = new_params.cl[0];
    this->bl = new_params.cl[1];
}

double MPC::CalculateControl(bool increase_pre, std::array<double, MPC_TIME_HORIZON> y_des, double scale)
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

    
 

    f_p1 = scale * this->UpdateF(p_h, p_l, this->u_his.begin() , *cur_a, *cur_b) + F_offset;
    f_p2 = scale * this->UpdateF(p_h + 1, p_l + 1, this->u_his.begin() + 1, *cur_a, *cur_b) + F_offset;
    f_p3 = scale * this->UpdateF(p_h + 2, p_l + 2, this->u_his.begin() + 2, *cur_a, *cur_b) + F_offset;
    f_p4 = scale * this->UpdateF(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b) + F_offset;
    f_p5 = scale * this->UpdateF(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b) + F_offset;
    f_p6 = scale * this->UpdateF(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b) + F_offset;
    f_p7 = scale * this->UpdateF(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b) + F_offset;
    f_p8 = scale * this->UpdateF(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b) + F_offset;
    f_p9 = scale * this->UpdateF(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b) + F_offset;

    df_dun_p1_T = scale * this->Update_dF_du_T(p_h, p_l, this->u_his.begin() + 0, *cur_a, *cur_b);
    df_dun_p2_T = scale * this->Update_dF_du_T(p_h + 1, p_l + 1, this->u_his.begin() + 1, *cur_a, *cur_b);
    df_dun_p3_T = scale * this->Update_dF_du_T(p_h + 2, p_l + 2, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun_p4_T = scale * this->Update_dF_du_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun_p5_T = scale * this->Update_dF_du_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun_p6_T = scale * this->Update_dF_du_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun_p7_T = scale * this->Update_dF_du_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun_p8_T = scale * this->Update_dF_du_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun_p9_T = scale * this->Update_dF_du_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun1_p2_T = scale * this->Update_dF_du1_T(p_h + 1, p_l + 1, this->u_his.begin() + 1, *cur_a, *cur_b);
    df_dun1_p3_T = scale * this->Update_dF_du1_T(p_h + 2, p_l + 2, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun1_p4_T = scale * this->Update_dF_du1_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun1_p5_T = scale * this->Update_dF_du1_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun1_p6_T = scale * this->Update_dF_du1_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun1_p7_T = scale * this->Update_dF_du1_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun1_p8_T = scale * this->Update_dF_du1_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun1_p9_T = scale * this->Update_dF_du1_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun2_p3_T = scale * this->Update_dF_du2_T(p_h + 2, p_l + 2, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dun2_p4_T = scale * this->Update_dF_du2_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun2_p5_T = scale * this->Update_dF_du2_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun2_p6_T = scale * this->Update_dF_du2_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun2_p7_T = scale * this->Update_dF_du2_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun2_p8_T = scale * this->Update_dF_du2_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun2_p9_T = scale * this->Update_dF_du2_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun3_p4_T = scale * this->Update_dF_du3_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dun3_p5_T = scale * this->Update_dF_du3_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun3_p6_T = scale * this->Update_dF_du3_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun3_p7_T = scale * this->Update_dF_du3_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun3_p8_T = scale * this->Update_dF_du3_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun3_p9_T = scale * this->Update_dF_du3_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun4_p5_T = scale * this->Update_dF_du4_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dun4_p6_T = scale * this->Update_dF_du4_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun4_p7_T = scale * this->Update_dF_du4_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun4_p8_T = scale * this->Update_dF_du4_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun4_p9_T = scale * this->Update_dF_du4_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun5_p6_T = scale * this->Update_dF_du5_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dun5_p7_T = scale * this->Update_dF_du5_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun5_p8_T = scale * this->Update_dF_du5_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun5_p9_T = scale * this->Update_dF_du5_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun6_p7_T = scale * this->Update_dF_du6_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dun6_p8_T = scale * this->Update_dF_du6_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun6_p9_T = scale * this->Update_dF_du6_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun7_p8_T = scale * this->Update_dF_du7_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dun7_p9_T = scale * this->Update_dF_du7_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dun8_p9_T = scale * this->Update_dF_du8_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn_p2_T = scale * this->Update_dF_dxH_T(p_h + 1, p_l + 1, this->u_his.begin() + 1, *cur_a, *cur_b);
    df_dxn_p3_T = scale * this->Update_dF_dxH_T(p_h + 2, p_l + 2, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dxn_p4_T = scale * this->Update_dF_dxH_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn_p5_T = scale * this->Update_dF_dxH_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn_p6_T = scale * this->Update_dF_dxH_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn_p7_T = scale * this->Update_dF_dxH_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn_p8_T = scale * this->Update_dF_dxH_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn_p9_T = scale * this->Update_dF_dxH_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn1_p3_T = scale * this->Update_dF_dxH1_T(p_h + 2, p_l + 2, this->u_his.begin() + 2, *cur_a, *cur_b);
    df_dxn1_p4_T = scale * this->Update_dF_dxH1_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn1_p5_T = scale * this->Update_dF_dxH1_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn1_p6_T = scale * this->Update_dF_dxH1_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn1_p7_T = scale * this->Update_dF_dxH1_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn1_p8_T = scale * this->Update_dF_dxH1_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn1_p9_T = scale * this->Update_dF_dxH1_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn2_p4_T = scale * this->Update_dF_dxH2_T(p_h + 3, p_l + 3, this->u_his.begin() + 3, *cur_a, *cur_b);
    df_dxn2_p5_T = scale * this->Update_dF_dxH2_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn2_p6_T = scale * this->Update_dF_dxH2_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn2_p7_T = scale * this->Update_dF_dxH2_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn2_p8_T = scale * this->Update_dF_dxH2_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn2_p9_T = scale * this->Update_dF_dxH2_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn3_p5_T = scale * this->Update_dF_dxH3_T(p_h + 4, p_l + 4, this->u_his.begin() + 4, *cur_a, *cur_b);
    df_dxn3_p6_T = scale * this->Update_dF_dxH3_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn3_p7_T = scale * this->Update_dF_dxH3_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn3_p8_T = scale * this->Update_dF_dxH3_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn3_p9_T = scale * this->Update_dF_dxH3_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn4_p6_T = scale * this->Update_dF_dxH4_T(p_h + 5, p_l + 5, this->u_his.begin() + 5, *cur_a, *cur_b);
    df_dxn4_p7_T = scale * this->Update_dF_dxH4_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn4_p8_T = scale * this->Update_dF_dxH4_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn4_p9_T = scale * this->Update_dF_dxH4_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn5_p7_T = scale * this->Update_dF_dxH5_T(p_h + 6, p_l + 6, this->u_his.begin() + 6, *cur_a, *cur_b);
    df_dxn5_p8_T = scale * this->Update_dF_dxH5_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn5_p9_T = scale * this->Update_dF_dxH5_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn6_p8_T = scale * this->Update_dF_dxH6_T(p_h + 7, p_l + 7, this->u_his.begin() + 7, *cur_a, *cur_b);
    df_dxn6_p9_T = scale * this->Update_dF_dxH6_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

    df_dxn7_p9_T = scale * this->Update_dF_dxH7_T(p_h + 8, p_l + 8, this->u_his.begin() + 8, *cur_a, *cur_b);

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

    Eigen::DiagonalMatrix<double,MPC_TIME_HORIZON> w_mat{5,1,1,1,1,1,1,1,1};

    Eigen::Matrix<double, MPC_TIME_HORIZON, MPC_TIME_HORIZON> P_mat = B_all.transpose() *w_mat* B_all;
    Eigen::Matrix<double, 1, MPC_TIME_HORIZON> q_mat = -1 * y_des_vec.transpose() *w_mat* B_all + A_all.transpose() *w_mat* B_all;

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

    c_int A_nnz = MPC_TIME_HORIZON+2;
    c_float A_x[MPC_TIME_HORIZON+16] = {1, 1,
                                       1, -1, 1,
                                       1, -1, 1,
                                       1, -1 ,1,
                                       1, -1, 1,
                                       1, -1, 1,
                                       1, -1, 1,
                                       1, -1, 1,
                                       1, -1};

    c_int A_i[MPC_TIME_HORIZON+16] = {0, 9,
                                     1, 9, 10,
                                     2, 10,11,
                                     3, 11,12,
                                     4, 12,13,
                                     5, 13,14,
                                     6, 14,15,
                                     7, 15,16,
                                     8, 16};
    c_int A_p[MPC_TIME_HORIZON + 1] = {0,  2, 5, 8, 11,14,17,20,23,25};
    c_float l[MPC_TIME_HORIZON+8] = {0.15, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0};
    c_float u[MPC_TIME_HORIZON+8] = {1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1,1,1};
    c_int n = MPC_TIME_HORIZON;
    c_int m = MPC_TIME_HORIZON+8;
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
    this->u_n5 = *(this->work->solution->x + 5);
    this->u_n6 = *(this->work->solution->x + 6);
    this->u_n7 = *(this->work->solution->x + 7);
    this->u_n8 = *(this->work->solution->x + 8);

    Eigen::Matrix<double, MPC_TIME_HORIZON, 1> u_vec;

    u_vec << this->u_n, this->u_n1, this->u_n2, this->u_n3, this->u_n4, this->u_n5, this->u_n6, this->u_n7, this->u_n8;


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
    // std::cout<<this->u_n<<std::endl;
    // this->cur_dF = scale * this->UpdateF(p_h, p_l, this->u_his.begin(), *cur_a, *cur_b);
    this->u_his[MPC_TIME_HORIZON] = this->u_n; //add the current duty to u_his for estimating gradient
    return this->u_n;
    // return (int)(this->u_n * 100 + 0.5);
}

int MPC::GetPreControl(const std::array<double,MPC_TIME_HORIZON> &p_des, const double &ps, const double &pt, double scale)
{

    double p_diff = (p_des[0] - ps); // we scaled the p_diff with the assumption that pressure will have the momentum to go

    if (std::abs(p_diff) > 100) // 640 is 2 psi
    {                           // if desired pressure has 1 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias
        // std::cout<<"err is large: "<<p_diff<<std::endl;
        // std::cout<<ps-pt<<std::endl;
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
        std::array<double, MPC_TIME_HORIZON> y_des;
        for(unsigned i=0;i<p_des.size();i++){
            y_des[i]=(p_des[i] - 3297.312) / 65536.0;
        }
        // std::cout<<"p_diff: "<<p_diff/65536<<std::endl;
        this->UpdateHistory(ps, pt,p_des[0]);

        // std::cout<<"current p_des scale: "<<p_des_scale<<std::endl;
        // std::cout<<"current p_cur scale: "<<this->p_set_his[MPC_TIME_HORIZON]<<std::endl;

        double ideal_u = 0;
        if ((p_des[0] > ps) & (pt > ps))// & (pt > p_des[0]))   //Even the p_des is not feasible, as long as we can improve the current condition, we should still try
        {
            // increasing pressure

            ideal_u = this->CalculateControl(true, y_des, scale);

            this->cur_dF = this->UpdateF(this->p_tank_his.begin(),this->p_set_his.begin(),this->u_his.begin()+1,this->ah,this->bh);
            
            
        }
        else if ((p_des[0] < ps) & (ps > pt))// & (p_des[0] > pt))
        {
            // decreasing pressure
            ideal_u = this->CalculateControl(false, y_des, scale);
            this->cur_dF = this->UpdateF(this->p_set_his.begin(),this->p_tank_his.begin(),this->u_his.begin()+1,this->al,this->bl);
        }
        else
        {
            this->cur_dF =this->cur_dF*0.5; //FIXME: why 0.5???
            ideal_u=0;
        }

        // std::cout<<"real p_diff: "<<this->Phi.coeff(1,0)*65536<<std::endl;

        // std::cout<<std::endl;
        // std::cout << "ideal duty: " << ideal_duty << std::endl;
        // std::cout << "control p_diff: " << (this->alpha.coeff(1, 0) + this->B.coeff(1, 0) * ideal_duty/100) * 65536 << std::endl;
        // std::cout << "des pdiff: " << p_diff << std::endl;

        if (ideal_u < 0.15)
        {
            this->cur_u = 0;
            
        }
        // else if (ideal_duty < 15 && ideal_duty > 10)
        // {
        //     // return 0;
        //     return 15;
        // }
        else if (ideal_u > 1)
            this->cur_u = 1;
        else
        {
            this->cur_u =ideal_u;
        }
    }
    else
    {
        
        // this->cur_F << 0, 0;
        this->cur_dF =this->cur_dF*0.5;
        this->u_n = 0;
        this->u_n1 = 0;
        this->u_n2 = 0;
        this->cur_u =0;
    }
    return (int)(this->cur_u*100+0.5);

}

void MPC::UpdateMeas(double p_set,double p_tank,u_int8_t duty)
{
    
    //we first need to calculate the gradient estimation from the nonlinear model,
    this->u_mem[this->meas_idx] = ((double)duty)/100.0;
    this->UpdateHistory(p_set, p_tank,p_set);
    Eigen::Matrix<double,2,1> pre_dF;
    if(p_tank>=p_set){
        pre_dF = this->UpdateF(this->p_tank_his.begin(),this->p_set_his.begin(),this->u_his.begin(),this->ah,this->bh);
    }
    else{
        pre_dF = this->UpdateF(this->p_set_his.begin(),this->p_tank_his.begin(),this->u_his.begin(),this->al,this->bl);
    }
    this->mpc_model_rec.PushData(std::array<double,2>{pre_dF.coeff(0),pre_dF.coeff(1)},1); //this is actually the rate of change in the previous time step


    this->p_tank_mem[this->meas_idx] = ((double)p_tank - 3297.312) / 65536.0; // the substraction is to remove the 0.5 V pressure sensor bias and add 1 atm to the equation
    this->p_set_mem[this->meas_idx] = ((double)p_set - 3297.312) / 65536.0;   // in the lasso regression, we have proved it increases the testing accuracy to stable 90% up
    
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

void MPC::UpdateHistory(double p_set, double p_tank,double p_des)
{
    std::memset(this->p_tank_his.begin(), 0, this->p_tank_his.size());
    std::memset(this->p_set_his.begin(), 0, this->p_set_his.size());
    std::memset(this->u_his.begin(), 0, this->u_his.size());

    double p_step = (p_des-p_set)/(double)MPC_TIME_HORIZON/65536.0;
    // std::cout<<"p_step: "<<p_step<<std::endl;

    for (int i = 0; i < MPC_DELAY; i++)
    {
        this->p_tank_his[i] = this->p_tank_mem[(this->meas_idx + i) % MPC_DELAY];
        this->p_set_his[i] = this->p_set_mem[(this->meas_idx + i) % MPC_DELAY];
        this->u_his[i] = this->u_mem[(this->meas_idx + i+1) % MPC_DELAY];
    }

    for (int i = 0; i < MPC_TIME_HORIZON; i++)
    {
        this->p_tank_his[i + MPC_DELAY] = (p_tank - 3297.312) / 65536-p_step*i*0.58993; //This literally has no meaning but let's give it a try
        this->p_set_his[i + MPC_DELAY] = (p_set - 3297.312) / 65536+p_step*i;
        this->u_his[i + MPC_DELAY] = MPC::kUBar; // use the lower bound first, in case the previous duty was 0


        // this->p_tank_his[i + MPC_DELAY] = (p_tank - 3297.312) / 65536; //This literally has no meaning but let's give it a try
        // this->p_set_his[i + MPC_DELAY] = (p_set - 3297.312) / 65536;
        // this->u_his[i + MPC_DELAY] = MPC::kUBar; // use the lower bound first, in case the previous duty was 0

    }




    // std::cout<<"mem values: "<<this->p_tank_his[0]<<std::endl;
}

