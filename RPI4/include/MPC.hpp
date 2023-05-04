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
#include "ExoConfig.hpp"

#define MPC_HEAD "Time,F_0,F_1,dF_0,dF_1,u_n,u_n1,u_n2,u_n3,u_n4,u_n5,u_n6,u_n7,u_n8,yn1,yn2,yn3,yn4,yn5,yn6,yn7,yn8,yn9,y_des1,y_des2,y_des3,y_des4,y_des5,y_des6,y_des7,y_des8,y_des9"
#define MPC_MODEL_HEAD "Time,dF_0,dF_1"
// #define MPC_DATA_HEAD "Time,A1_0,A1_1,,A2_0,A2_1,A3_0,A3_1,A4_0,A4_1,A5_0,A5_1,A6_0,A6_1,A7_0,A7_1,A8_0,A8_1,A9_0,A9_1"
#define MPC_TIME_HORIZON 9
class MPC
{
private:
    // Please reference to the equation note

    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 1> alpha;
    double CalculateControl(bool increase_pre, std::array<double, MPC_TIME_HORIZON> y_des, double scale);
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
    std::array<double,MPC_TIME_HORIZON> y_des;

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
    double cur_u; //     duty/100

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
    // double phi_scale;

    // const double volume_slope_6in = 0.0006351973436310972;  // FIXME: these are only used for linear calibrations
    // const double volume_intercept_6in = 115.68133521647316; // unit: mm/adc(pos)

    // const double spring_k = 55.4 * 0.0393701 * 4.44822;
    // const double pre_offset = 0.5 / 4.096 * 65536;
    Recorder<double, 31> mpc_rec;
    // Recorder<double,2> mpc_model_rec; //this recorder records the real gradient estimation from the nonlinear model
    

public:
    MPC(ExoConfig::MPC_Params mpc_params,std::string file_name);
    ~MPC();

    ExoConfig::MPC_Params mpc_params;
    double GetMpcCalibLen(); //get the chamber length when calculating the mpc parameters
    void UpdateParam(ExoConfig::MPC_Params new_params);
    int GetPreControl(const std::array<double, MPC_TIME_HORIZON> &p_des, const double &p_cur, const double &p_tank, double scale); // It requires current pressure value because all the values storaged in the meme are scaled
    void GetImpControl(const double &imp_des, const double &p_ext, const double &p_flex, const double &p_tank, const double &pos, double scale, int &joint_val_duty, int &joint_bal_duty, int &tank_duty);
    // Get values for recorder

    void UpdateMeas(double,double,u_int8_t);
    void RecData();

    // double GetCylinderScale(double pre,double pos); //get the (cylinder length)/(max cylinder length)
    // void SetCylinderMaxPos();
};

#endif