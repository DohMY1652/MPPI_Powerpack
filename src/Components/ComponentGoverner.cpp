#include "mppi_brl/Components/ComponentGoverner.h"

ComponentGoverner::ComponentGoverner(std::shared_ptr<DatabaseConfig>& databaseconfig)
: databaseconfig(databaseconfig)  {
    control.resize(databaseconfig->get_n_control(), 0.0);
    flow_rate.resize(databaseconfig->get_n_control(), 0.0);
    state.resize(databaseconfig->get_n_state(), 101.325);
    state[-1] = 0;
    state_dot.resize(databaseconfig->get_n_state(), 0.0);

    dt = databaseconfig->get_dt();
    Pmacro = databaseconfig->get_P_macro();

    valve_orifice_diameter = databaseconfig->get_valve_parameters()[0];
    ejector_orifice_diameter = databaseconfig->get_ejector_parameters()[0];
    actuator_positive_area = databaseconfig->get_actuator_parameters()[0];
    actuator_negative_area = databaseconfig->get_actuator_parameters()[1];
    actuator_stroke = databaseconfig->get_actuator_parameters()[2];
    external_load = databaseconfig->get_actuator_parameters()[3];
    
    pump = std::make_shared<Pump>(databaseconfig->get_pump_parameters());
    valve = std::make_shared<Valve>(databaseconfig->get_valve_parameters());
    ejector = std::make_shared<Ejector>(databaseconfig->get_ejector_parameters());
    actuator = std::make_shared<Actuator>(databaseconfig->get_actuator_parameters());
    tank = std::make_shared<Tank>(databaseconfig->get_tank_parameters());

}

ComponentGoverner::~ComponentGoverner() {}

bool ComponentGoverner::set_control(std::vector<double> _control) {
    if (_control.size() != control.size()) {
        ROS_WARN("Wrong control dimension");
        return false;
    }
    else {
        control = _control;
        return true;
    }
}

bool ComponentGoverner::set_state(std::vector<double> _state) { 
    if (_state.size() != state.size()) {
        ROS_WARN("Wrong state dimension");
        return false;
    }
    else {
        state = _state;
        return true;
    }
}

void ComponentGoverner::calculate_flow_rate() {
    flow_rate[0] = pump->calculate_flow_rate(control[0], state[1], state[0]); // kg/s
    flow_rate[1] = valve->calculate_flow_rate(control[1], state[0], Patm); // kg/s
    flow_rate[2] = valve->calculate_flow_rate(control[2], Patm, state[1]); // kg/s
    flow_rate[3] = valve->calculate_flow_rate(control[3], state[0], state[2]); // kg/s
    flow_rate[4] = valve->calculate_flow_rate(control[4], state[2], Patm); // kg/s
    flow_rate[5] = valve->calculate_flow_rate(control[5], Pmacro, state[2]); // kg/s
    flow_rate[6] = valve->calculate_flow_rate(control[6], state[3], state[1]); // kg/s
    flow_rate[7] = valve->calculate_flow_rate(control[7], Patm, state[3]); // kg/s
    flow_rate[8] = valve->calculate_flow_rate(1.0, state[3], ejector->get_P_suction()); // kg/s
    // flow_rate[8] = ejector->calculate_suction_flow_rate(state[3]); // kg/s
}

void ComponentGoverner::calculate_state_dot() {
    state_dot[0] = tank->calculate_pressure_dot(pump->calculate_flow_rate_out(control[0], state[1],state[0]), flow_rate[1]+flow_rate[3]); // kPa/s
    state_dot[1] = tank->calculate_pressure_dot(flow_rate[2]+flow_rate[6], pump->calculate_flow_rate_in(control[0], state[1],state[0])); // kPa/s
    state_dot[2] = tank->calculate_pressure_dot(flow_rate[3]+flow_rate[5], flow_rate[4]);
    state_dot[3] = tank->calculate_pressure_dot(flow_rate[7], flow_rate[6]+flow_rate[8]);
    state_dot[4] = 0;
    // state_dot[4] = actuator->calculate_length_dot(state[2], state[3], state[4]);
    // state_dot[2] = actuator->calculate_positive_pressure_dot(flow_rate[3]+flow_rate[5],flow_rate[4]);
    // state_dot[3] = actuator->calculate_negative_pressure_dot(flow_rate[7], flow_rate[6]+flow_rate[8]);
}


void ComponentGoverner::calculate_state() {
    for (int i =0; i < state.size(); ++i) {
        state[i] = state[i] + state_dot[i] * dt;
    }
    if(state[4] <= 0) { state[4] = 0;}
    if(state[4] >= 0.020) { state[4] = 0.020;}
    pump->update_P_and_theta(dt);
    actuator->set_positive_pressure(state[2]);
    actuator->set_negative_pressure(state[3]);
    if (control[8] != 0.0) {
        ejector->calculate_comsuming_flow_rate(); // kg/s
    }
    else {
        ejector->set_comsuming_flow_rate(valve->calculate_flow_rate(1.0, ejector->get_P_ejector(), Patm));
    }
    ejector->update_P_ejector(valve->calculate_flow_rate(control[8], Pmacro, ejector->get_P_ejector()));
    ejector->update_P_suction();
}

void ComponentGoverner::calculate_next_state(std::vector<double> _control) {
   
    if (set_control(_control)) {
        calculate_flow_rate();
        calculate_state_dot();
        calculate_state();
    }
}

std::vector<double> ComponentGoverner::get_state() const {
    return state;
}

std::vector<double> ComponentGoverner::get_flow_rate() const {
    return flow_rate;
}

std::vector<double> ComponentGoverner::get_state_dot() const {
    return state_dot;
}


double ComponentGoverner::get_force() const {
    return actuator->get_force();
}