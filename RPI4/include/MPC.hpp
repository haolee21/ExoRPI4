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
    void UpdateDyn(float p_h,float p_l,float d,bool increase_pre);
    //parameter of OSQP
    float P_val,q_val;
    const Eigen::Matrix<float,1,2> H_h; // when I define state, I define it as 
    const Eigen::Matrix<float,1,2> H_l;
    Eigen::Matrix<float,2,1> Phi; //this will be useful if we want to estimate the flow rate
    Eigen::Matrix2f dPhi_dx;
    Eigen::Matrix<float,2,1> dPhi_du;
    void UpdatePhi(float ph,float pl,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);
    void Update_dPhi_dxL(float ph,float pl,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);
    void Update_dPhi_duL(float ph,float pl,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);

    void Update_dPhi_dxH(float ph,float pl,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);
    void Update_dPhi_duH(float ph,float pl,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b);

    // Eigen::Matrix2f matA;
    // Eigen::Matrix2f matA2;
    // Eigen::Matrix2f matA3;
    // Eigen::Matrix<float,2,1> matB;
    // const Eigen::Matrix<float,1,2> phi{1,0};

    
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
    
    int GetControl(int p_des,int p_tank,int p_set,int duty);

    std::array<float,2> GetPhi();
 
};




#endif