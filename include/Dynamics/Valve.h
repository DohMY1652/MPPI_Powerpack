#ifndef VALVE_H
#define VALVE_H

#include "Dynamics/FlowControlComponent.h"

#include <math.h>

class Valve : public FlowControlComponent {
    public:
        Valve(const std::vector<double>& system_parameters);
        virtual double calculate_flow_rate(double control, double Pin, double Pout) override;


    private:
        const double R = 0.287 ; // kJ/(kg*K)
        const double T = 293.15 ; // K
        const double kappa = 1.4 ;
        
        double CsvDpid;
        double Pr;
        double Pcr;
        double pi_function;

        const double Patm = 101.325 ; // kPa
        const double Cu = 4.876 ; // mm^2
        const double Cp = 3.066 ; //mm^2/N
        const double Ck = 4.474 ; // mm^2   

};

#endif // VALVE_H