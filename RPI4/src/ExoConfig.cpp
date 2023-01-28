#include "ExoConfig.hpp"
#include <iostream>
#include <fstream>
using json = nlohmann::json;
// using namespace ExoConfig;
template <class T, unsigned N>
std::array<T, N> LoadJsonArray(json json_file, std::string name)
{
    std::array<T, N> result;
    auto json_array = json_file[name];
    if (N > json_array.size())
    {
        std::cout << "JSON Read Error: json array " << name << " size mismatch (required " << N << " members)\n";
    }
    else
    {
        for (unsigned i = 0; i < N; i++)
        {
            result[i] = json_array[i];
        }
    }

    return result;
}
json ExoConfig::LoadJson(std::string file_name)
{
    try
    {
        json file;
        std::ifstream ifs(file_name);
        file = json::parse(ifs);
        return file;
    }
    catch(const std::exception& e)
    {
        std::cerr<<file_name<<" does not exist/loading errors\n";
        // std::cerr << e.what() << '\n';
        return 0;
    }
    
    

    
}
void ExoConfig::LoadPhyParam(std::string name,CylnPhyParams& phy_param,const nlohmann::json &config_file){
    phy_param.cyln_eqn = LoadJsonArray<double,3>(config_file[name],"CylnEqn");
    phy_param.chamber_max_len = config_file[name]["CylnChamberMaxLen"];
    phy_param.mech_max_len = config_file[name]["CylnMaxMechLen"];
    phy_param.fri_coeff = config_file[name]["FrictionCoeff"];
    phy_param.piston_area = LoadJsonArray<double,2>(config_file[name],"PistonArea_mm2");
    phy_param.spring_const = config_file[name]["SpringConst"];
    phy_param.neutral_pos = config_file[name]["NeutralPos"];
}
void ExoConfig::LoadMPC_param(const nlohmann::json &config_file,MPC_Params &mpc_params,std::string name){
    mpc_params.ch[0]= LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["ch"], "a");
    mpc_params.ch[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["ch"], "b");
    mpc_params.cl[0] = LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["cl"], "a");
    mpc_params.cl[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["cl"], "b");
    mpc_params.cali_chamber_len = config_file[name]["calib_len"];
}
void ExoConfig::LoadConfigFile(const json &config_file,ExoConfig::SystemParam &sys_param,  ExoConfig::ConfigType config_type)
{
    
    // ExoConfig::SystemParam &sys_param = std::ref(ExoConfig::GetInstance().sys_param);
    // auto sys_param = ExoConfig::SystemParam();


    if (config_type ==ExoConfig::ConfigType::MPC)
    {
        std::cout<<"Load MPC Config\n";
        ExoConfig::LoadMPC_param(config_file,sys_param.left_tank_subtank,"LTankSubtank");
        ExoConfig::LoadMPC_param(config_file,sys_param.left_subtank_knee,"LTankKnee");
        ExoConfig::LoadMPC_param(config_file,sys_param.left_subtank_ank,"LTankAnk");
        ExoConfig::LoadMPC_param(config_file,sys_param.left_knee_ank,"LKneeAnk");
        ExoConfig::LoadMPC_param(config_file,sys_param.left_knee_ext_flex,"LKneeExtFlex");
        ExoConfig::LoadMPC_param(config_file,sys_param.left_ank_ext_flex,"LAnkExtFlex");

        
        ExoConfig::LoadMPC_param(config_file,sys_param.right_tank_subtank,"RTankSubtank");
        ExoConfig::LoadMPC_param(config_file,sys_param.right_subtank_knee,"RTankKnee");
        ExoConfig::LoadMPC_param(config_file,sys_param.right_subtank_ank,"RTankAnk");
        ExoConfig::LoadMPC_param(config_file,sys_param.right_knee_ank,"RKneeAnk");
        ExoConfig::LoadMPC_param(config_file,sys_param.right_knee_ext_flex,"RKneeExtFlex");
        ExoConfig::LoadMPC_param(config_file,sys_param.right_ank_ext_flex,"RAnkExtFlex");

        std::cout << "size: " << config_file["LTankSubtank"]["ch"]["a"].size() << std::endl;
    }
    else if(config_type==ExoConfig::ConfigType::Phy){
        //cylinder length equation
        std::cout<<"Load Phy Config\n";
        ExoConfig::LoadPhyParam("LeftKnee",sys_param.left_knee_phy,config_file);
        ExoConfig::LoadPhyParam("LeftAnkle",sys_param.left_ankle_phy,config_file);
        ExoConfig::LoadPhyParam("RightKnee",sys_param.right_knee_phy,config_file);
        ExoConfig::LoadPhyParam("RightAnkle",sys_param.right_ankle_phy,config_file);
    }
    else{
        std::cout<<"ExoConfig type wrong\n";
    }
    
    // std::cout << config_file["Name"] <<"  "<< config_file["ConfigVersion"] << std::endl;
    std::cout << "done loading\n";
}
ExoConfig::ExoConfig()
{
    //load default config files
    std::cout<<"begin config file loading\n";
    auto exo_config_file = ExoConfig::LoadJson("exo_config.json");
    ExoConfig::LoadConfigFile(exo_config_file,this->sys_param,  ExoConfig::ConfigType::Phy);
    std::cout<<"end config file loading\n";
    auto config_file = ExoConfig::LoadJson("mpc_config.json");
    ExoConfig::LoadConfigFile(config_file,this->sys_param, ExoConfig::ConfigType::MPC);
}
ExoConfig::~ExoConfig()
{

}
ExoConfig& ExoConfig::GetInstance(){
    static ExoConfig instance;
    return instance;
}
const ExoConfig::SystemParam& ExoConfig::GetConfig(){
    return std::cref(ExoConfig::GetInstance().sys_param);
}
// void ExoConfig::SaveConfigFile(std::string file_name,SystemParam sys_param){
//     json new_config;

// }