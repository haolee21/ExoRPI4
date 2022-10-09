/**
 * The MPC controller class controls the pressure in the "set" cylinder
 * The connection is: tank <=> set
 *
 * Four possible condition
 *
 * P_tank > P_set, and P_des > P_slave: use parameters ch and UpdateH to generate desired control
 * P_tank < P_set, and P_des > P_slave: output 0
 * P_tank < P_set, and P_des < P_slave: use parameters cl and updateL to generate desired control
 * P_tank < P_set, and P_des > P_slave: output 0
 *
 *
 * The unit will be SI unit:
 * length: mm
 * pressure: kPa
 * force: N
 **/

#ifndef MPC_H
#define MPC_H

#include <math.h>
#include <memory>
#include <array>
#include <unsupported/Eigen/MatrixFunctions>
#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <string>

#include "MPC_param.hpp"
#include "Recorder.hpp"
#include "DigitalFilter.hpp"
#include "FilterParam.hpp"
#include "CylinderParam.hpp"

#define MPC_HEAD "Time,F_0,F_1,dF_0,dF_1,u_n,u_n1,u_n2,u_n3,u_n4,u_n5,u_n6,u_n7,u_n8,yn1,yn2,yn3,yn4,yn5,yn6,yn7,yn8,yn9,y_des1,y_des2,y_des3,y_des4,y_des5,y_des6,y_des7,y_des8,y_des9"
#define MPC_TIME_HORIZON 9
class MPC
{
private:
    // Please reference to the equation note

    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 1> alpha;
    int DutyCalculate(bool increase_pre, std::array<double, MPC_TIME_HORIZON> y_des, double scale);
    // parameter of OSQP

    const double kUBar = 0.15; // FIXME: test if making it not the lower bound improve the performance
    // const Eigen::Matrix<double,1,2> H_h; // when I define state, I define it as
    // const Eigen::Matrix<double,1,2> H_l;
    Eigen::Matrix<double, 2, 1> cur_F; // this will be useful if we want to estimate the flow rate
    Eigen::Matrix<double, 2, 1> cur_dF;
    double u_n, u_n1, u_n2, u_n3, u_n4, u_n5, u_n6, u_n7, u_n8; // only u_n is used, but we should also record u_n1 and u_n2
    Eigen::Matrix<double, 2, 1> x_n1;
    Eigen::Matrix<double, 2, 1> x_n2;
    Eigen::Matrix<double, 2, 1> x_n3;
    Eigen::Matrix<double, 2, 1> x_n4;
    Eigen::Matrix<double, 2, 1> x_n5;
    Eigen::Matrix<double, 2, 1> x_n6;
    Eigen::Matrix<double, 2, 1> x_n7;
    Eigen::Matrix<double, 2, 1> x_n8;
    Eigen::Matrix<double, 2, 1> x_n9;
    double y_des1, y_des2, y_des3, y_des4, y_des5, y_des6, y_des7, y_des8, y_des9;

    Eigen::Matrix<double, 2, 1> UpdateF(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxL_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL1_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH1_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du1_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL2_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH2_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du2_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL3_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH3_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du3_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL4_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH4_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du4_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL5_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH5_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du5_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL6_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH6_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du6_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL7_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH7_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du7_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 2> Update_dF_dxL8_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 2> Update_dF_dxH8_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);
    Eigen::Matrix<double, 2, 1> Update_dF_du8_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    Eigen::Matrix<double, 2, 1> Update_dF_du9_T(const double *ph, const double *pl, const double *u, const std::array<double, MPC_STATE_NUM> &a, const std::array<double, MPC_STATE_NUM> &b);

    // Mem for previous measurements
    std::array<double, MPC_DELAY> p_tank_mem; // mem is just for storage, pop the oldest ones and put the newest one there, the order may be 34512
    std::array<double, MPC_DELAY> p_set_mem;
    std::array<double, MPC_DELAY> u_mem;

    std::array<double, MPC_DELAY + MPC_TIME_HORIZON> p_tank_his;
    std::array<double, MPC_DELAY + MPC_TIME_HORIZON> p_set_his;
    std::array<double, MPC_DELAY + MPC_TIME_HORIZON> u_his;

    // std::array<double,MPC_DELAY> p_tank_hat;
    // std::array<double,MPC_DELAY> p_set_hat;
    // std::array<double,MPC_DELAY> u_hat;
    // void SortHistory();
    void UpdateHistory(double p_set, double p_tank, double p_des);

    unsigned meas_idx;

    // the cylinders are divided into master and slave
    // we control the
    std::array<double, MPC_STATE_NUM> ah;
    std::array<double, MPC_STATE_NUM> bh;
    std::array<double, MPC_STATE_NUM> al;
    std::array<double, MPC_STATE_NUM> bl;

    // generate MPC constants

    std::unique_ptr<OSQPSettings> osqp_settings;
    std::unique_ptr<OSQPData> osqp_data;
    OSQPWorkspace *work;
    // bool mpc_enable;

    // cylinder volume

    // static const double kArea;//= 0.31*645.16; //mm^2
    double phi_scale;

    const double volume_slope_6in = 0.0006351973436310972;  // FIXME: these are only used for linear calibrations
    const double volume_intercept_6in = 115.68133521647316; // unit: mm/adc(pos)

    const double spring_k = 55.4 * 0.0393701 * 4.44822;
    // the unit here is N/mm
    const double pre_offset = 0.5 / 4.096 * 65536;
    // const double piston_area;
    // double GetExternalForce(double p_ext,double p_flex,double x);
    // double GetLenLinear_mm(double pos);
    // piston friction compensation
    // double pre_pos;
    // double pos_diff;
    // double cur_pos;
    // double fric_coeff;

    // double cur_max_spring_compress;
    // double cur_force;
    // DigitalFilter<double,FilterParam::Filter20Hz_5::Order,1> vel_filter;
    // DigitalFilter<double,FilterParam::Filter20Hz_2::Order,1> force_filter;
    Recorder<double, 31> mpc_rec;

public:
    MPC(std::array<std::array<double, MPC_STATE_NUM>, 2> cl, std::array<std::array<double, MPC_STATE_NUM>, 2> ch, std::string file_name);
    ~MPC();
    void UpdateParamH(std::array<double, MPC_STATE_NUM> new_param0, std::array<double, MPC_STATE_NUM> new_param1);
    void UpdateParamL(std::array<double, MPC_STATE_NUM> new_param0, std::array<double, MPC_STATE_NUM> new_param1);

    int GetPreControl(const std::array<double, MPC_TIME_HORIZON> &p_des, const double &p_cur, const double &p_tank, double scale); // It requires current pressure value because all the values storaged in the meme are scaled
    void GetImpControl(const double &imp_des, const double &p_ext, const double &p_flex, const double &p_tank, const double &pos, double scale, int &joint_val_duty, int &joint_bal_duty, int &tank_duty);
    // Get values for recorder

    void PushMeas(const double p_tank, const double p_set, const uint8_t duty);
    void RecData();

    // double GetCylinderScale(double pre,double pos); //get the (cylinder length)/(max cylinder length)
    // void SetCylinderMaxPos();
};

#endif