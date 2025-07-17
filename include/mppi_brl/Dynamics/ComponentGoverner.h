#ifndef COMPONENTGOVERNER_H
#define COMPONENTGOVERNER_H

#include "mppi_brl/DynamicsDatabaseConfig.h"

#include "mppi_brl/Dynamics/Actuator.h"
#include "mppi_brl/Dynamics/Tank.h"
#include "mppi_brl/Dynamics/Pump.h"
#include "mppi_brl/Dynamics/Valve.h"
#include "mppi_brl/Dynamics/Ejector.h"

class ComponentGoverner {
    public:
        ComponentGoverner(std::shared_ptr<DynamicsDatabaseConfig>& databaseconfig);
        ~ComponentGoverner();

        
        bool set_control(std::vector<double> _control);
        bool set_state(std::vector<double> _state);
        void calculate_flow_rate();
        void calculate_state_dot();
        void calculate_state();
        
        void calculate_next_state(std::vector<double> _control);

        std::vector<double> get_state() const;
        std::vector<double> get_flow_rate() const;
        std::vector<double> get_state_dot() const;
        double get_force() const;

    private:
        std::shared_ptr<DynamicsDatabaseConfig>& databaseconfig;

        std::vector<double> control;
        std::vector<double> flow_rate;
        std::vector<double> state;
        std::vector<double> state_dot;
        std::vector<double> force;

        double dt;
        const double Patm = 101.325; // kPa
        const double R = 287; // J/(kg * K)
        const double T = 293.15; // K
        double Pmacro;

        double valve_orifice_diameter;
        double ejector_orifice_diameter;
        double actuator_positive_area;
        double actuator_negative_area;
        double actuator_stroke;
        double external_load;

        double flow_rate_ejector_in;
        double ejector_suction_pressure;
        double ejector_comsuming_flow_rate;

        std::shared_ptr<Pump> pump;
        std::shared_ptr<Valve> valve;
        std::shared_ptr<Ejector> ejector;
        std::shared_ptr<Actuator> actuator;
        std::shared_ptr<Tank> tank;
};


#endif // COMPONENTGOVERNER_H
