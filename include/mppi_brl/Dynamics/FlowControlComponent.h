#ifndef FLOWCONTROLCOMPONENT_H
#define FLOWCONTROLCOMPONENT_H

#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>


class FlowControlComponent {
    public:
        FlowControlComponent(std::vector<double> system_parameters);
        ~FlowControlComponent();

        double get_flow_rate();
        double get_orifice_diameter();
        void set_inlet_pressure(double _inlet_pressure);
        void set_outlet_pressure(double _outlet_pressure);
        void set_control(double _control);

        virtual double calculate_flow_rate(double control, double Pin, double Pout) =0;

    protected:
    
        double orifice_diameter;

        double flow_rate;
        double inlet_pressure;
        double outlet_pressure;
        double control;
};


#endif // FLOWCONTROLCOMPONENT_H