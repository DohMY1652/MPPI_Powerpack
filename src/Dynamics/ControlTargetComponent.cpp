#include "Dynamics/ControlTargetComponent.h"

ControlTargetComponent::ControlTargetComponent(std::vector<double> system_parameters)
 : input_flow_rate(0.0), output_flow_rate(0.0), pressure(101.325), 
   pressure_dot(0.0), length(0.0), length_dot(0.0) {}

ControlTargetComponent::~ControlTargetComponent() {}

double ControlTargetComponent::get_pressure() {
    return pressure;
}


void ControlTargetComponent::set_input_flow_rate(double _input_flow_rate) {
    input_flow_rate = _input_flow_rate;
}

void ControlTargetComponent::set_output_flow_rate(double _output_flow_rate) {
    output_flow_rate = _output_flow_rate;
}

void ControlTargetComponent::set_system_parameters(std::vector<double> system_parameters) {

}