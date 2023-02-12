// #ifndef VALVECON_HPP
// #define VALVECON_HPP
// #include "MPC.hpp"
// #include "CylinderParam.hpp"
// class ValveCon
// {
// private:
//     MPC mpc;
// public:
//     ValveCon(CylinderParam::Params cyln_params);
//     ~ValveCon();
//     void SetImp(double imp);
//     void SetPre(const double &p_set,const double &p_cur,const double &p_src,const double scale);
// };

// ValveCon::ValveCon(CylinderParam::Params cyln_params)
// :mpc(cyln_params.cl,cyln_params.ch,cyln_params.max_pos,cyln_params.fri_coeff)
// {
// }

// ValveCon::~ValveCon()
// {
// }

// void ValveCon::SetPre(const double &p_set,const double &p_cur,const double &p_src,const double scale){
//     this->mpc.GetPreControl(p_set,p_cur,p_src,scale);
// }
// #endif