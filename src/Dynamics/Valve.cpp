#include "Dynamics/Valve.h"

Valve::Valve(const std::vector<double>& sysetm_parameters)
 : FlowControlComponent(sysetm_parameters) {
    orifice_diameter = sysetm_parameters[0];
 }


double Valve::calculate_flow_rate(double control, double Pin, double Pout) {
    CsvDpid = std::max(Cu*control+Cp*M_PI*pow(orifice_diameter,2)*(Pin-Pout)*0.25*0.001-Ck,0.0);
    Pr = Pout/Pin;
    Pcr = pow((2/(kappa+1)),(kappa/(kappa-1)));

    if (Pr <= Pcr) {
        pi_function = sqrt(kappa*pow((2/(kappa+1)),((kappa+1)/(kappa-1))));
    } else if (Pr <= 1) {
        pi_function = sqrt((2*kappa)/(kappa-1))*sqrt(pow(Pr, (2/kappa))-pow(Pr, ((kappa+1)/kappa)));
    } else {
        pi_function = 0;
    }

    return CsvDpid*Pin/sqrt(R*T)*pi_function * 3.16 * 0.00001;
    // scaling : Ccv [mm^2], Pin [kPa], R [kJ/(kg*K)], T [K], pi_function [None]
    // 1 mm^2*kPa/sqrt(kJ/kg) = 3.16 * 10^-5 kg/s
}