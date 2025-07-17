#ifndef CONTROLTARGETCOMPONENT_H
#define CONTROLTARGETCOMPONENT_H

#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>


class ControlTargetComponent {
    public:
        ControlTargetComponent(std::vector<double> system_parameters);
        ~ControlTargetComponent();


        double get_pressure();
        void set_input_flow_rate(double _input_flow_rate);
        void set_output_flow_rate(double _output_flow_rate);

        virtual double calculate_pressure_dot(double flow_rate_in, double flow_rate_out) =0;
    

    protected:

        virtual void set_system_parameters(std::vector<double> system_parameters);

        const double R = 0.287; //kJ/(kg*K)
        const double T = 293.15; // K
        
        double input_flow_rate;
        double output_flow_rate;

        double pressure;
        double pressure_dot;
        double length;
        double length_dot;

};


#endif // CONTROLTARGETCOMPONENT_H