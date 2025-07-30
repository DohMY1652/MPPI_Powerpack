#ifndef DATABASECONFIG_H
#define DATABASECONFIG_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <vector>

class DatabaseConfig {
   public:
    DatabaseConfig(const YAML::Node& config);
    ~DatabaseConfig();

    double get_dt() const;
    int get_N() const;
    int get_K() const;
    double get_tracking_gain() const;

    double get_lambda() const;

    double get_P_macro() const;

    int get_n_control() const;
    int get_n_state() const;

    double get_simulation_frequency() const;

    std::vector<double> get_pump_parameters() const;
    std::vector<double> get_valve_parameters() const;
    std::vector<double> get_ejector_parameters() const;
    std::vector<double> get_actuator_parameters() const;
    std::vector<double> get_tank_parameters() const;


    std::vector<double> get_distribution_parameters() const;
    std::vector<double> get_weights() const;

    
   private:
    YAML::Node config;
    
    double dt;
    int N;
    int K;
    double tracking_gain;

    double lambda;

    double P_macro;
    int n_control;
    int n_state;

    std::vector<double> pump_parameters;
    std::vector<double> valve_parameters;
    std::vector<double> ejector_parameters;
    std::vector<double> actuator_parameters;
    std::vector<double> tank_parameters;

    std::vector<double> distribution_parameters;
    std::vector<double> weights;

    double simulation_frequency;

};

#endif  // DATABASECONFIG_H