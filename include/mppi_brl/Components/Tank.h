#ifndef TANK_H
#define TANK_H

#include "mppi_brl/Components/ControlTargetComponent.h"

class Tank: public ControlTargetComponent {
    public:

        Tank(const std::vector<double>& system_parameters);
        double calculate_pressure_dot(double flow_rate_in, double flow_rate_out) override;

    private:
        

        double volume;
};

#endif // TANK_H