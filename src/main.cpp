#include "ros/ros.h"

#include "mppi_brl/DynamicsDatabaseConfig.h"
#include "mppi_brl/Dynamics/ComponentGoverner.h"
#include "mppi_brl/Controller/MPPI.h"


#include <vector>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <string>


double system_time = 0.0;

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

    std::string dynamics_parameters_yaml_file;
    if (!nh.getParam("dynamics_parameters_yaml_file", dynamics_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'dynamics_parameters_yaml_file'");
        return 1;
    }
    YAML::Node dynamics_config = YAML::LoadFile(dynamics_parameters_yaml_file);
    
    std::string visualizing_parameters_yaml_file;
    if (!nh.getParam("visualizing_parameters_yaml_file", visualizing_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'visualizing_parameters_yaml_file'");
        return 1;
    }
    YAML::Node visualzing_config = YAML::LoadFile(visualizing_parameters_yaml_file);

    // std::string controller_parameters_yaml_file;
    // if (!nh.getParam("controller_parameters_yaml_file", controller_parameters_yaml_file)) {
    //     ROS_ERROR("Could not find parameter 'controller_parameters_yaml_file'");
    //     return 1;
    // }
    // YAML::Node controller_config = YAML::LoadFile(visualizing_parameters_yaml_file);
    
    std::shared_ptr<DynamicsDatabaseConfig> dynamics_databaseconfig = std::make_shared<DynamicsDatabaseConfig>(dynamics_config);
    // std::shared_ptr<DatabaseConfig> controller_databaseconfig = std::make_shared<DatabaseConfig>(controller_config);
    std::shared_ptr<ComponentGoverner> dynamics = std::make_shared<ComponentGoverner>(dynamics_databaseconfig);
    // std::shared_ptr<MPPI> controller = std::make_shared<MPPI>(controller_databaseconfig);
    
    
    ros::Rate rate(1/dynamics_databaseconfig->get_dt());
    // ros::Rate rate(100000);

    if(!dynamics->set_state({101.325, 101.325, 101.325, 101.325, 0.0})) {
        return 1;
    }
    
    while (ros::ok()) {
        ros::spinOnce();
        ros::Time now = ros::Time::now();
        system_time += dynamics_databaseconfig->get_dt();
        dynamics->calculate_next_state({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        if ((now - last_trigger_time) >= trigger_interval) {
            last_trigger_time = now;
            

            std::cout << "Time : " << system_time << "\t ";
            print_vector(dynamics->get_state(), "states");
            // print_vector(dynamics->get_flow_rate(), "flow_rate");
            // print_vector(dynamics->get_state_dot(), "state_dot");
            // std::cout << "force : " << dynamics->get_force() <<std::endl;
            
        }
        rate.sleep();
    }


    return 0;
}
