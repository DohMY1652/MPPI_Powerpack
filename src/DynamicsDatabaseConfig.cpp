#include "mppi_brl/DynamicsDatabaseConfig.h"

DynamicsDatabaseConfig::DynamicsDatabaseConfig(const YAML::Node& config)
     : config(config), distribution_parameters({0.00, 0.00}) {
    dt = config["system_parameters"]["dt"].as<double>();
    N = config["system_parameters"]["prediction_horizon"].as<int>();
    K = config["system_parameters"]["sampling_number"].as<int>(); 

    distribution_parameters[0] = config["distribution_parameters"]["mean"].as<double>();
    distribution_parameters[1] = config["distribution_parameters"]["variance"].as<double>();

    lambda = config["inverse_temperature"].as<double>();

    P_macro = config["P_Macro"].as<double>();

    n_control = config["n_control"].as<int>();
    n_state = config["n_state"].as<int>();

    simulation_frequency = config["simulation_parameters"]["frequency"].as<double>();

    valve_parameters.push_back(config["Valve_parameters"]["Orifice_diameter"].as<double>());
    pump_parameters.push_back(config["Pump_parameters"]["Orifice_diameter"].as<double>());
    ejector_parameters.push_back(config["Ejector_parameters"]["Orifice_diameter"].as<double>());
    ejector_parameters.push_back(config["Ejector_parameters"]["dt"].as<double>());


    actuator_parameters.push_back(config["Actuator_parameters"]["length_of_neg_octagon"].as<double>());
    actuator_parameters.push_back(config["Actuator_parameters"]["radius_of_load_shaft"].as<double>());
    actuator_parameters.push_back(config["Actuator_parameters"]["External_load"].as<double>());
    actuator_parameters.push_back(config["Actuator_parameters"]["L_max"].as<double>());
    actuator_parameters.push_back(config["Actuator_parameters"]["dt"].as<double>());
    actuator_parameters.push_back(config["Actuator_parameters"]["positive_diameter"].as<double>());
    actuator_parameters.push_back(config["Actuator_parameters"]["positive_length"].as<double>());

    tank_parameters.push_back(config["Tank_parameters"]["volume"].as<double>());

    weights.push_back(config["weights"]["A"].as<double>());


}

DynamicsDatabaseConfig::~DynamicsDatabaseConfig() {}

double DynamicsDatabaseConfig::get_dt() const { return dt; }
int DynamicsDatabaseConfig::get_N() const { return N; }
int DynamicsDatabaseConfig::get_K() const { return K; }

double DynamicsDatabaseConfig::get_lambda() const { return lambda; }

double DynamicsDatabaseConfig::get_P_macro() const {return P_macro;}

int DynamicsDatabaseConfig::get_n_control() const {return n_control; }
int DynamicsDatabaseConfig::get_n_state() const {return n_state;}

double DynamicsDatabaseConfig::get_simulation_frequency() const {return simulation_frequency;}

std::vector<double> DynamicsDatabaseConfig::get_pump_parameters() const { return pump_parameters; }
std::vector<double> DynamicsDatabaseConfig::get_valve_parameters() const { return valve_parameters; }
std::vector<double> DynamicsDatabaseConfig::get_ejector_parameters() const { return ejector_parameters; }
std::vector<double> DynamicsDatabaseConfig::get_actuator_parameters() const { return actuator_parameters; }
std::vector<double> DynamicsDatabaseConfig::get_tank_parameters() const { return tank_parameters; }



std::vector<double> DynamicsDatabaseConfig::get_distribution_parameters() const { return distribution_parameters; }
std::vector<double> DynamicsDatabaseConfig::get_weights() const { return weights; }
