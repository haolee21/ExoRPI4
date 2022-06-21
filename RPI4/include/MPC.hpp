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
 * 
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

class MPC
{
private:
    // Please reference to the equation note
    
    
    Eigen::Matrix<float,2,1> B;
    Eigen::Matrix<float,2,1> alpha;
    void UpdateDyn(bool increase_pre);
    //parameter of OSQP
    float P_val,q_val;
    const Eigen::Matrix<float,1,2> H_h; // when I define state, I define it as 
    const Eigen::Matrix<float,1,2> H_l;
    Eigen::Matrix<float,2,1> Phi; //this will be useful if we want to estimate the flow rate
    Eigen::Matrix<float,2,2> dPhi_dx_T;
    Eigen::Matrix<float,2,1> dPhi_du_T;
    void UpdatePhi(const std::array<float,MPC_DELAY> ph,const std::array<float,MPC_DELAY> pl,const std::array<float,MPC_DELAY> u,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);
    void Update_dPhi_dxL(const std::array<float,MPC_DELAY>& ph, const std::array<float,MPC_DELAY> &pl,const std::array<float,MPC_DELAY> &d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);
    void Update_dPhi_dxH(const std::array<float,MPC_DELAY>& ph,const std::array<float,MPC_DELAY> &pl,const std::array<float,MPC_DELAY>& d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);
    void Update_dPhi_du(const std::array<float,MPC_DELAY>& ph,const std::array<float,MPC_DELAY> &pl,const std::array<float,MPC_DELAY>& d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);

    //Mem for previous measurements
    std::array<float,MPC_DELAY> p_tank_mem; //mem is just for storage, pop the oldest ones and put the newest one there, the order may be 34512
    std::array<float,MPC_DELAY> p_set_mem;
    std::array<float,MPC_DELAY> u_mem;

    std::array<float,MPC_DELAY> p_tank_his;
    std::array<float,MPC_DELAY> p_set_his;
    std::array<float,MPC_DELAY> u_his;

    std::array<float,MPC_DELAY> p_tank_hat;
    std::array<float,MPC_DELAY> p_set_hat;
    std::array<float,MPC_DELAY> u_hat;
    // void SortHistory();
    void UpdateHistory();

    unsigned meas_idx;

    
    // the cylinders are divided into master and slave
    // we control the 
    std::array<float,MPC_STATE_NUM> ah;
    std::array<float,MPC_STATE_NUM> bh;
    std::array<float,MPC_STATE_NUM> al;
    std::array<float,MPC_STATE_NUM> bl;



    //generate the OSQP constants


    std::unique_ptr<OSQPSettings> osqp_settings;
    std::unique_ptr<OSQPData> osqp_data;
    OSQPWorkspace *work;

    bool mpc_enable;

    
    
public:
    MPC(std::array<std::array<float,MPC_STATE_NUM>,2> init_cl,std::array<std::array<float,MPC_STATE_NUM>,2> init_ch);
    ~MPC();
    void UpdateParamH(std::array<float,MPC_STATE_NUM> new_param0,std::array<float,MPC_STATE_NUM> new_param1);
    void UpdateParamL(std::array<float,MPC_STATE_NUM> new_param0,std::array<float,MPC_STATE_NUM> new_param1);
    
    int GetControl(const u_int16_t& p_des,const u_int16_t& p_cur,const u_int16_t& p_tank);//It requires current pressure value because all the values storaged in the meme are scaled

    //Get values for recorder
    std::array<float,10> GetMpcRec();

    void PushPreMeas(const u_int16_t p_tank,const u_int16_t p_set,const u_int16_t duty);
 
};




#endif