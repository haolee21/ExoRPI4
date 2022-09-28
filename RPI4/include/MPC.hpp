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

#define LIN_CONST_LEN 2

#include<math.h>
#include<memory>
#include<array>
#include <unsupported/Eigen/MatrixFunctions>
#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <string>


#include "MPC_param.hpp"
#include "Recorder.hpp"
#include "DigitalFilter.hpp"
#include "FilterParam.hpp"
#include "CylinderParam.hpp"

#define MPC_HEAD "Time,P_diff_des,Phi_1,Phi_2,Phi_hat_1,Phi_hat_2,dPhi1_dx1,dPhi1_dx2,dPhi2_dx1,dPhi2_dx2,dPhi1_du,dPhi2_du"
class MPC
{
private:
    // Please reference to the equation note
    
    
    Eigen::Matrix<double,2,1> B;
    Eigen::Matrix<double,2,1> alpha;
    void UpdateDyn(bool increase_pre);
    //parameter of OSQP
    double P_val,q_val;
    double p_diff_des; //TODO: remove this after verifying control

    const Eigen::Matrix<double,1,2> H_h; // when I define state, I define it as 
    const Eigen::Matrix<double,1,2> H_l;
    Eigen::Matrix<double,2,1> Phi; //this will be useful if we want to estimate the flow rate
    Eigen::Matrix<double,2,1> Phi_hat;
    Eigen::Matrix<double,2,2> dPhi_dx_T;
    Eigen::Matrix<double,2,1> dPhi_du_T;

    Eigen::Matrix<double,2,2> dPhi_dx2_T;
    Eigen::Matrix<double,2,1> dPhi_du2_T;
    Eigen::Matrix<double,2,2> dPhi_dx3_T;
    Eigen::Matrix<double,2,1> dPhi_du3_T;

    Eigen::Matrix<double,2,2> dPhi_dx4_T;
    Eigen::Matrix<double,2,1> dPhi_du4_T;
    Eigen::Matrix<double,2,2> dPhi_dx5_T;
    Eigen::Matrix<double,2,1> dPhi_du5_T;


    void UpdatePhi(const std::array<double,MPC_DELAY> ph,const std::array<double,MPC_DELAY> pl,const std::array<double,MPC_DELAY> u,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b,Eigen::Matrix<double,2,1>& _phi);
    void Update_dPhi_dxL(const std::array<double,MPC_DELAY>& ph, const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY> &d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_dxH(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_du(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);

    void Update_dPhi_dxL2(const std::array<double,MPC_DELAY>& ph, const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY> &d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_dxH2(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_du2(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    
    void Update_dPhi_dxL3(const std::array<double,MPC_DELAY>& ph, const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY> &d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_dxH3(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_du3(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);

    void Update_dPhi_dxL4(const std::array<double,MPC_DELAY>& ph, const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY> &d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_dxH4(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_du4(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);

    void Update_dPhi_dxL5(const std::array<double,MPC_DELAY>& ph, const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY> &d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_dxH5(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);
    void Update_dPhi_du5(const std::array<double,MPC_DELAY>& ph,const std::array<double,MPC_DELAY> &pl,const std::array<double,MPC_DELAY>& d,const std::array<double,MPC_STATE_NUM>& a,const std::array<double,MPC_STATE_NUM> &b);

    //Mem for previous measurements
    std::array<double,MPC_DELAY> p_tank_mem; //mem is just for storage, pop the oldest ones and put the newest one there, the order may be 34512
    std::array<double,MPC_DELAY> p_set_mem;
    std::array<double,MPC_DELAY> u_mem;

    std::array<double,MPC_DELAY> p_tank_his;
    std::array<double,MPC_DELAY> p_set_his;
    std::array<double,MPC_DELAY> u_his;

    std::array<double,MPC_DELAY> p_tank_hat;
    std::array<double,MPC_DELAY> p_set_hat;
    std::array<double,MPC_DELAY> u_hat;
    // void SortHistory();
    void UpdateHistory();

    unsigned meas_idx;

    
    // the cylinders are divided into master and slave
    // we control the 
    std::array<double,MPC_STATE_NUM> ah;
    std::array<double,MPC_STATE_NUM> bh;
    std::array<double,MPC_STATE_NUM> al;
    std::array<double,MPC_STATE_NUM> bl;



    
    //generate MPC constants

    // std::unique_ptr<OSQPSettings> osqp_settings;
    // std::unique_ptr<OSQPData> osqp_data;
    // OSQPWorkspace *work;
    // bool mpc_enable;

    //cylinder volume
    
    // static const double kArea;//= 0.31*645.16; //mm^2
    double phi_scale;
    
    const double volume_slope_6in = 0.0006351973436310972; //FIXME: these are only used for linear calibrations
    const double volume_intercept_6in = 115.68133521647316; // unit: mm/adc(pos)


    const double spring_k = 55.4*0.0393701*4.44822; 
                                         // the unit here is N/mm
    const double pre_offset = 0.5/4.096*65536;
    // const double piston_area;
    // double GetExternalForce(double p_ext,double p_flex,double x);
    // double GetLenLinear_mm(double pos);
    //piston friction compensation
    // double pre_pos;
    // double pos_diff;
    // double cur_pos;
    // double fric_coeff;
    
    // double cur_max_spring_compress;
    // double cur_force;
    // DigitalFilter<double,FilterParam::Filter20Hz_5::Order,1> vel_filter;
    // DigitalFilter<double,FilterParam::Filter20Hz_2::Order,1> force_filter;
    Recorder<double,11> mpc_rec;
public:
    MPC(std::array<std::array<double, MPC_STATE_NUM>,2> cl,std::array<std::array<double, MPC_STATE_NUM>,2> ch,std::string file_name);
    ~MPC();
    void UpdateParamH(std::array<double,MPC_STATE_NUM> new_param0,std::array<double,MPC_STATE_NUM> new_param1);
    void UpdateParamL(std::array<double,MPC_STATE_NUM> new_param0,std::array<double,MPC_STATE_NUM> new_param1);

    
    int GetPreControl(const double& p_des,const double& p_cur,const double& p_tank,double scale);//It requires current pressure value because all the values storaged in the meme are scaled
    void GetImpControl(const double& imp_des, const double& p_ext,const double& p_flex, const double& p_tank, const double& pos,double scale,int& joint_val_duty,int& joint_bal_duty,int& tank_duty);
    //Get values for recorder
    std::array<double,10> GetMpcRec();

    void PushMeas(const double p_tank,const double p_set,const uint8_t duty);
    
    
    
    // double GetCylinderScale(double pre,double pos); //get the (cylinder length)/(max cylinder length)
    // void SetCylinderMaxPos();


};




#endif