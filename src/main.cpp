#include "ros/ros.h"

#include "mppi_brl/DatabaseConfig.h"
#include "mppi_brl/Components/ComponentGoverner.h"


#include <vector>
#include <algorithm>
#include <math.h>

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
    ros::init(argc, argv, "mppi_brl");
    ros::NodeHandle nh;

    ros::Time last_trigger_time = ros::Time::now();
    ros::Duration trigger_interval(0.2);

    std::string control_parameters_yaml_file;
    if (!nh.getParam("control_parameters_yaml_file", control_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'control_parameters_yaml_file'");
        return 1;
    }
    YAML::Node control_config = YAML::LoadFile(control_parameters_yaml_file);
    
    std::string visualizing_parameters_yaml_file;
    if (!nh.getParam("visualizing_parameters_yaml_file", visualizing_parameters_yaml_file)) {
        ROS_ERROR("Could not find parameter 'visualizing_parameters_yaml_file'");
        return 1;
    }
    YAML::Node visualzing_config = YAML::LoadFile(visualizing_parameters_yaml_file);
    
    std::shared_ptr<DatabaseConfig> control_databaseconfig = std::make_shared<DatabaseConfig>(control_config);
    std::shared_ptr<ComponentGoverner> dynamics = std::make_shared<ComponentGoverner>(control_databaseconfig);
    
    ros::Rate rate((int)(1/control_databaseconfig->get_dt()));
    // ros::Rate rate(10);

    if(!dynamics->set_state({101.325, 101.325, 101.325, 101.325, 0.0})) {
        return 1;
    }
    
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        system_time += control_databaseconfig->get_dt();
        if (system_time < 5) {
        //     dynamics->calculate_next_state({1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0});
        // } else if (system_time < 20) {
            dynamics->calculate_next_state({0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0});
        } else {
            dynamics->calculate_next_state({0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0});
        }
        // dynamics->calculate_next_state({1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        if ((now - last_trigger_time) >= trigger_interval) {
            last_trigger_time = now;

            std::cout << "Time : " << system_time << "\t ";
            print_vector(dynamics->get_state(), "states");
            print_vector(dynamics->get_flow_rate(), "flow_rate");
            // print_vector(dynamics->get_state_dot(), "state_dot");
            // std::cout << "force : " << dynamics->get_force() <<std::endl;
            
        }
        rate.sleep();
        // std::cout <<"running" << std::endl;
    }


    return 0;
}

