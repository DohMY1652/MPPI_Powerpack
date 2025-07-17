#include "mppi_brl/Dynamics/Pump.h"


Pump::Pump(const std::vector<double>& sysetm_parameters)
 : FlowControlComponent(sysetm_parameters) {
    orifice_diameter = sysetm_parameters[0];
 }

double Pump::calculate_flow_rate(double control, double Pin, double Pout) {
    return (flow_rate_in+flow_rate_out)/2.0;
}

double Pump::calculate_flow_rate_in(double control, double Pin, double Pout) {
    if (control > 0.5) {
        calculate_V_and_V_dot();
        if (V_dot_1 >= 0) {
            Pr = P_1/Pin;
        } else {
            Pr = P_2/Pin;
        }
        Pcr = pow((2/(kappa+1)),(kappa/(kappa-1)));

        if (Pr <= Pcr) {
            pi_function = sqrt(kappa*pow((2/(kappa+1)),((kappa+1)/(kappa-1))));
        } else if (Pr <= 1) {
            pi_function = sqrt((2*kappa)/(kappa-1))*sqrt(pow(Pr, (2/kappa))-pow(Pr, ((kappa+1)/kappa)));
        } else {
            pi_function = 0;
        }
        if (flow_rate_in <= 0) {
            flow_rate_in = 0;
        }

        flow_rate_in = Ccv_in*Pin/sqrt(R*T)*pi_function * 3.16 * 0.00001;
        // scaling : Ccv [mm^2], Pin [kPa], R [kJ/(kg*K)], T [K], pi_function [None]
        // 1 mm^2*kPa/sqrt(kJ/kg) = 3.16 * 10^-5 kg/s
    } else {
        flow_rate_in = 0.0;
    }

    return flow_rate_in;
    
}

double Pump::calculate_flow_rate_out(double control, double Pin, double Pout) {
    if (control >= 0.5) {
        calculate_V_and_V_dot();

        if (V_dot_1 <= 0) {
            Pr = Pout/P_1;
        } else {
            Pr = Pout/P_2;
        }

        Pcr = pow((2/(kappa+1)),(kappa/(kappa-1)));

        if (Pr <= Pcr) {
            pi_function = sqrt(kappa*pow((2/(kappa+1)),((kappa+1)/(kappa-1))));
        } else if (Pr <= 1) {
            pi_function = sqrt((2*kappa)/(kappa-1))*sqrt(pow(Pr, (2/kappa))-pow(Pr, ((kappa+1)/kappa)));
        } else {
            pi_function = 0;
        }
        if (flow_rate_out <= 0) {
            flow_rate_out = 0;
        }

        flow_rate_out = Ccv_out*P_1/sqrt(R*T)*pi_function * 3.16 * 0.00001;
        // scaling : Ccv [mm^2], Pin [kPa], R [kJ/(kg*K)], T [K], pi_function [None]
        // 1 mm^2*kPa/sqrt(kJ/kg) = 3.16 * 10^-5 kg/s
    } else {
        flow_rate_out = 0.0;
    }
   
    return flow_rate_out;
}



void Pump::calculate_V_and_V_dot() {
    V_1 = Spis*(delta-r+l-r*cos(theta_1)-sqrt(l*l-r*r*sin(theta_1)*sin(theta_1))); // m^3
    V_dot_1 = Spis*omega_1*(r*sin(theta_1)+(r*r*sin(theta_1)*cos(theta_1))/(sqrt(l*l-r*r*sin(theta_1)*sin(theta_1)))); // m^3/s
    V_2 = Spis*(delta-r+l-r*cos(theta_2)-sqrt(l*l-r*r*sin(theta_2)*sin(theta_2))); // m^3
    V_dot_2 = Spis*omega_2*(r*sin(theta_2)+(r*r*sin(theta_2)*cos(theta_2))/(sqrt(l*l-r*r*sin(theta_2)*sin(theta_2)))); // m^3/s

}


void Pump::update_P_and_theta(double dt) {
    theta_1 = theta_1 + omega_1 * dt;
    theta_2 = theta_2 + omega_2 * dt;
    if(V_dot_1 >= 0) {
        P_1 = (P_1+ -1*((P_1* V_dot_1-flow_rate_in*R*T)/V_1) * dt); // kPa
        P_2 = (P_2+ -1*((P_2* V_dot_2+flow_rate_out*R*T)/V_2) * dt); // kPa
    } else {
        P_1 = (P_1+ -1*((P_1* V_dot_1+flow_rate_out*R*T)/V_1) * dt); // kPa
        P_2 = (P_2+ -1*((P_2* V_dot_2-flow_rate_in*R*T)/V_2) * dt); // kPa
    }
    if (P_1 <= 0) {
        P_1 = 0;
    }
    if (P_2 <= 0) {
        P_2 = 0;
    }

}