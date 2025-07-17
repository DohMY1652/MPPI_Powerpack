#ifndef EJECTOR_H
#define EJECTOR_H

#include "mppi_brl/Components/FlowControlComponent.h"

class Ejector : public FlowControlComponent {
    public:
        Ejector(const std::vector<double>& system_parameters);
        double calculate_flow_rate(double control, double Pin, double Pout) override;

        double get_P_ejector() const;
        double get_P_suction() const;

        void update_P_ejector(double flow_rate_in);
        void calculate_comsuming_flow_rate();
        void update_P_suction();
        
        void set_comsuming_flow_rate(double _comsuming_flow_rate);
    private :
        double P_ejector;
        double P_suction;
        double comsuming_flow_rate = 0.0;
        double dt;
        double orifice_diameter;

        const double V = 0.000758150605; //m^3
        const double R = 0.287 ; // kJ/(kg*K)
        const double T = 293.15 ; // K        

        double interpolate(double p, const std::vector<double>& values) const;

        const std::vector<double> pressures_kPa = {0.0, 100.0, 200.0, 300.0, 400.0};
        std::vector<double> suction_m3s;
        std::vector<double> consumption_m3s;
        std::vector<double> suction_pressure;
};

#endif // EJECTOR_H