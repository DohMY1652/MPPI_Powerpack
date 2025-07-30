#include "ros/ros.h"

#include "DatabaseConfig.h"
#include "Dynamics/ComponentGoverner.h"
#include "Controller/MPPI.h"


#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <string>


double system_time = 0.0;
std::vector<double> target = {201.325, 51.325, 181.325, 51.325, 0};

template <typename T>
void print_vector(const std::vector<T>& vec, const std::string& name = "vector") {
    std::cout << name << " = [";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i < vec.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamics");
    ros::NodeHandle nh;
    ros::Time last_trigger_time = ros::Time::now();
    ros::Duration trigger_interval(0.01);

    std::string system_parameters_yaml_file;
    if (!nh.getParam("system_parameters_yaml_file", system_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'system_parameters_yaml_file'");
        return 1;
    }
    YAML::Node system_config = YAML::LoadFile(system_parameters_yaml_file);
    
    std::string visualizing_parameters_yaml_file;
    if (!nh.getParam("visualizing_parameters_yaml_file", visualizing_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'visualizing_parameters_yaml_file'");
        return 1;
    }
    YAML::Node visualzing_config = YAML::LoadFile(visualizing_parameters_yaml_file);
    
    std::shared_ptr<DatabaseConfig> databaseconfig = std::make_shared<DatabaseConfig>(system_config);
    std::shared_ptr<ComponentGoverner> sampling_system = std::make_shared<ComponentGoverner>(databaseconfig);
    std::shared_ptr<ComponentGoverner> target_system = std::make_shared<ComponentGoverner>(databaseconfig);

    std::shared_ptr<MPPI> controller = std::make_shared<MPPI>(databaseconfig, sampling_system);
    
    
    // ros::Rate rate(1/databaseconfig->get_dt());
    ros::Rate rate(1);

    if(!target_system->set_state({101.325, 101.325, 101.325, 101.325, 0.0})) {
        return 1;
    }
    

    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();
        system_time += databaseconfig->get_dt();
        target_system->calculate_next_state(controller->calculate_control(target, target_system->get_state()));
        target_system->update_pump_state(databaseconfig->get_dt());
        // target_system->calculate_next_state({1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        if ((now - last_trigger_time) >= trigger_interval) {
            last_trigger_time = now;
            

            std::cout << "Time : " << system_time << "\t ";
            print_vector(target_system->get_state(), "states");
            // print_vector(dynamics->get_flow_rate(), "flow_rate");
            // print_vector(dynamics->get_state_dot(), "state_dot");
            // std::cout << "force : " << dynamics->get_force() <<std::endl;
            
        }
        // rate.sleep();
    }


    return 0;
}
