#include "mppi_brl/Dynamics/Tank.h"


Tank::Tank(const std::vector<double>& system_parameters)
    : ControlTargetComponent(system_parameters)  {
        volume = 0.000001 * system_parameters[0]; //mL to m^3
    }


double Tank::calculate_pressure_dot(double flow_rate_in, double flow_rate_out) {

    return R*T/volume*(flow_rate_in-flow_rate_out);

}
