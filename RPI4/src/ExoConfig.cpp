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
void ExoConfig::LoadMPC_param(const nlohmann::json &config_file,MPC_Params &mpc_params,std::string name){
    mpc_params.ch[0]= LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["ch"], "a");
    mpc_params.ch[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["ch"], "b");
    mpc_params.cl[0] = LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["cl"], "a");
    mpc_params.cl[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file[name]["cl"], "b");
    mpc_params.cali_chamber_len = config_file[name]["calib_len"];
}
void ExoConfig::LoadConfigFile(const json &config_file, SystemParam &sys_param, ConfigType config_type)
{
    if (config_type == ConfigType::MPC)
    {
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
    else if(config_type==ConfigType::Phy){
        //cylinder length equation
        sys_param.left_knee_phy.cyln_eqn = LoadJsonArray<double,3>(config_file["CylnEqn"],"LeftKnee");
        sys_param.left_ankle_phy.cyln_eqn = LoadJsonArray<double,3>(config_file["CylnEqn"],"LeftAnkle");
        sys_param.right_knee_phy.cyln_eqn = LoadJsonArray<double,3>(config_file["CylnEqn"],"RightKnee");
        sys_param.right_ankle_phy.cyln_eqn = LoadJsonArray<double,3>(config_file["CylnEqn"],"RightAnkle");

        //Max cylinder chamber length
        sys_param.left_knee_phy.chamber_max_len = config_file["CylnChamberMaxLen"]["LeftKnee"];
        sys_param.left_ankle_phy.chamber_max_len = config_file["CylnChamberMaxLen"]["LeftAnkle"];
        sys_param.right_knee_phy.chamber_max_len = config_file["CylnChamberMaxLen"]["RightKnee"];
        sys_param.right_ankle_phy.chamber_max_len = config_file["CylnChamberMaxLen"]["RightAnkle"];
        
        // Mech max length (spring+etc)
        sys_param.left_knee_phy.mech_max_len = config_file["CylnMaxMechLen"]["LeftKnee"];
        sys_param.left_ankle_phy.mech_max_len = config_file["CylnMaxMechLen"]["LeftAnkle"];
        sys_param.right_knee_phy.mech_max_len = config_file["CylnMaxMechLen"]["RightKnee"];
        sys_param.right_ankle_phy.mech_max_len = config_file["CylnMaxMechLen"]["RightAnkle"];
        

        //Friction coeff
        sys_param.left_knee_phy.fri_coeff = config_file["FrictionCoeff"]["LeftKnee"];
        sys_param.left_ankle_phy.fri_coeff = config_file["FrictionCoeff"]["LeftAnkle"];
        sys_param.right_knee_phy.fri_coeff = config_file["FrictionCoeff"]["RightKnee"];
        sys_param.right_ankle_phy.fri_coeff = config_file["FrictionCoeff"]["RightAnkle"];
        
        //Piston area
        sys_param.left_knee_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"LeftKnee");
        sys_param.left_ankle_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"LeftAnkle");
        sys_param.right_knee_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"RightKnee");
        sys_param.right_ankle_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"RightAnkle");
        std::cout<<"done loading cylinder length\n";
        //Spring constant
        sys_param.left_knee_phy.spring_const = config_file["SpringConst"]["LeftKnee"];
        sys_param.left_ankle_phy.spring_const = config_file["SpringConst"]["LeftAnkle"];
        sys_param.right_knee_phy.spring_const = config_file["SpringConst"]["RightKnee"];
        sys_param.right_ankle_phy.spring_const = config_file["SpringConst"]["RightAnkle"];
        //Neutral position, at these points, impedance =0
        sys_param.left_knee_phy.neutral_pos = config_file["NeutralPos"]["LeftKnee"];
        sys_param.left_ankle_phy.neutral_pos = config_file["NeutralPos"]["LeftAnkle"];
        sys_param.right_knee_phy.neutral_pos = config_file["NeutralPos"]["RightKnee"];
        sys_param.right_ankle_phy.neutral_pos = config_file["NeutralPos"]["RightAnkle"];
    }
    std::cout << config_file["Name"] <<"  "<< config_file["ConfigVersion"] << std::endl;
    std::cout << "done loading\n";
}
ExoConfig::ExoConfig()
{
    //load default config files
    
    auto config_file = ExoConfig::LoadJson("mpc_config.json");
    ExoConfig::LoadConfigFile(config_file,this->sys_param,ExoConfig::ConfigType::MPC);
    auto exo_config_file = ExoConfig::LoadJson("exo_config.json");
    ExoConfig::LoadConfigFile(exo_config_file,this->sys_param,ExoConfig::ConfigType::Phy);

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