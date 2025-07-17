#ifndef PUMP_H
#define PUMP_H

#include "mppi_brl/Components/FlowControlComponent.h"

#include <math.h>

class Pump : public FlowControlComponent {
    public:
        Pump(const std::vector<double>& system_parameters);
        double calculate_flow_rate(double control, double Pin, double Pout) override;
        double calculate_flow_rate_in(double control, double Pin, double Pout);
        double calculate_flow_rate_out(double control, double Pin, double Pout);
        void update_P_and_theta(double dt);

    private:
        const double R = 0.287 ; // kJ/(kg*K)
        const double T = 293.15 ; // K
        const double kappa = 1.4 ;

        double dt;
        
        double Pr;
        double Pcr;
        double pi_function;

        const double Patm = 101.325; // kPa
        // const double Ccv_out = 1.46; // mm^2   
        const double Ccv_out = 33.47; // mm^2   
        const double Ccv_in = 33.47; // mm^2   

        const double delta = 0.041; // m
        const double r = 0.02; // m
        const double l = 0.07; // m
        const double Spis = 39.485 * 0.0001; //m^2

        double flow_rate_in = 0; // LPM
        double flow_rate_out = 0; // LPM
        
        double theta_1 = 0; // rad
        double omega_1 = 314.159265; //rad/s = 3,000 rpm
        double P_1 = 101.325; //kPa
        double V_1;
        double V_dot_1;

        double theta_2 = M_PI; // rad
        double omega_2 = 314.159265; //rad/s = 3,000 rpm
        double P_2 = 101.325; //kPa
        double V_2;
        double V_dot_2;

        void calculate_V_and_V_dot();
};

#endif // PUMP_H