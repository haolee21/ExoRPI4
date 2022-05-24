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


class MPC
{
private:

    Eigen::Matrix2f matA;
    // Eigen::Matrix2f matA2;
    // Eigen::Matrix2f matA3;
    Eigen::Matrix<float,2,1> matB;
    const Eigen::Matrix<float,1,2> phi{1,0};

    
    // the cylinders are divided into master and slave
    // we control the 
    std::array<std::array<float,13>,2> ch;  
    std::array<std::array<float,13>,2> cl;


    void UpdateH(int p_t,int p_s,int duty); //generate the OSQP constants
    void UpdateL(int p_t,int p_s,int duty);

    std::unique_ptr<OSQPSettings> osqp_settings;
    std::unique_ptr<OSQPData> osqp_data;
    OSQPWorkspace *work;

    bool mpc_enable;
    
public:
    MPC(/* args */);
    ~MPC();
    void UpdateParamH(std::array<float,13> new_param0,std::array<float,13> new_param1);
    void UpdateParamL(std::array<float,13> new_param0,std::array<float,13> new_param1);
    
    int GetControl(int p_des,int p_tank,int p_set,int duty);

 
};




#endif