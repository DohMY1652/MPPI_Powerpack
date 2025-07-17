#include "mppi_brl/Components/FlowControlComponent.h"

FlowControlComponent::FlowControlComponent(std::vector<double> system_parameters) 
 : orifice_diameter(0.0), flow_rate(0.0), inlet_pressure(0.0),
 outlet_pressure(0.0), control(0.0) {}

FlowControlComponent::~FlowControlComponent() {}


double FlowControlComponent::get_flow_rate() {
    return flow_rate;
}

double FlowControlComponent::get_orifice_diameter() {
    return orifice_diameter;
}

void FlowControlComponent::set_inlet_pressure(double _inlet_pressure) {
    inlet_pressure = _inlet_pressure;
}

void FlowControlComponent::set_outlet_pressure(double _outlet_pressure) {
    outlet_pressure = _outlet_pressure;
}

void FlowControlComponent::set_control(double _control) {
    control = _control;
}