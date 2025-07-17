#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mppi_brl/Dynamics/ControlTargetComponent.h"

#include <math.h>

class Actuator : public ControlTargetComponent {
    public :
        Actuator(const std::vector<double>& system_parameters);
        double calculate_pressure_dot(double flow_rate_in, double flow_rate_out) override;
        double calculate_positive_pressure_dot(double flow_rate_in, double flow_rate_out);
        double calculate_negative_pressure_dot(double flow_rate_in, double flow_rate_out);
        double calculate_length_dot(double positive_pressure, double negative_pressure, double x);

        void set_positive_pressure(double pressure);
        void set_negative_pressure(double pressure);
        
        std::vector<double> get_L_table() const;
        std::vector<double> get_theta_table() const;
        double get_force() const;
        
        private :
        
        std::vector<double> l_table;
        std::vector<double> theta_table;
        
        
        void calculate_positive_and_negative_volume(double x, double x_dot);
        void build_lookup_table();
        double f(double theta, double l0, double l);
        double df(double theta, double l0);
        double solve_theta(double l0, double l);
        double lookup_theta(double l);
        double theta0 = 1.0;
        double tol    = 1e-8;
        int    maxIter= 100;

        double pos_volume;
        double neg_volume;
        double surface_area;
        const double L_min = 0;
        double L_max;
        double x_dot = 0;
        double theta_dot;
        double dt;
        double external_load;
        double F_tot;
        double V_tot;

        const double P_atm = 101.325;
        const double l_0 = 0.020;  // m
        const double del_l = 0.000001;  // m
        const double l_max = 0.020; //m
        const double l_min = 0.020*(1-0.3634); // m
        const double g = 9.81; //gravity acceleration

        double theta; 

        double D;
        double r;
        double positive_D;
        double positive_L;


        double V_pos;
        double V_pos_dot;
        double V_neg;
        double V_neg_dot;

        double P_pos;
        double P_neg;

};


#endif //ACTUATOR_H
