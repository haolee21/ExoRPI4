#ifndef MPC_H
#define MPC_H

#define LIN_CONST_LEN 2
#include<math.h>
#include<memory>

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
    float c00,c02,c03,c06,c07,c08,c09;
    float c12,c13,c16,c17,c18,c19;
    void UpdateA(int p_tank,int p_lt,int duty);

    std::unique_ptr<OSQPSettings> osqp_settings;
    std::unique_ptr<OSQPData> osqp_data;
    OSQPWorkspace *work;

    bool mpc_enable;
    
public:
    MPC(/* args */);
    ~MPC();
    void UpdateParam(float c00,float c02,float c03,float c06,float c07,float c08,float c09,float c12,float c13,float c16,float c17,float c18,float c19);
    
    int GetControl(int p_des,int p_tank,int p_lt,int duty);

 
};




#endif