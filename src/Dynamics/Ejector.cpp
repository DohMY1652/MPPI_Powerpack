#include "mppi_brl/Dynamics/Ejector.h"

Ejector::Ejector(const std::vector<double>& sysetm_parameters)
 : FlowControlComponent(sysetm_parameters) {
    orifice_diameter = sysetm_parameters[0];
    P_ejector = 101.325;
    dt = sysetm_parameters[1];

    // LPM -> m^3/s 변환 상수
    const double LPM_to_m3s = 1e-3 / 60.0;

    // m_suction 데이터 (LPM)
    std::vector<double> suction_LPM = {0.0, 31.0, 64.0, 90.0, 107.0};
    for (double f : suction_LPM) {
        suction_m3s.push_back(f * LPM_to_m3s);
    }

    // m_consumption 데이터 (LPM)
    std::vector<double> consumption_LPM = {0.0, 12.8, 34.6, 46.23, 57.53};
    for (double f : consumption_LPM) {
        consumption_m3s.push_back(f * LPM_to_m3s);
    }

    suction_pressure = {101.325, 80.325, 69.325, 44.325, 11.325};

 }


double Ejector::calculate_flow_rate(double control, double Pin, double Pout) {
    return 0;
}






double Ejector::get_P_ejector() const {
    return P_ejector;
}

double Ejector::get_P_suction() const {
    return P_suction;
}

void Ejector::update_P_ejector(double flow_rate_in) {
    double P_ejector_dot = ((flow_rate_in-comsuming_flow_rate) * (R) * T) / V;
    P_ejector = P_ejector + P_ejector_dot * dt;
}

void Ejector::update_P_suction() {
    P_suction = interpolate(P_ejector-101.325, suction_pressure);
}


void Ejector::calculate_comsuming_flow_rate() {
    comsuming_flow_rate = P_ejector / (R * T) * interpolate(P_ejector-101.325, consumption_m3s); // kg/s
    if (comsuming_flow_rate <= 0.0)
    {
        comsuming_flow_rate = 0.0;
    }
}

void Ejector::set_comsuming_flow_rate(double _comsuming_flow_rate) {
    comsuming_flow_rate = _comsuming_flow_rate;
}


// private

double Ejector::interpolate(double p, const std::vector<double>& values) const {
    auto it = std::lower_bound(pressures_kPa.begin(), pressures_kPa.end(), p);
    size_t idx;
    if (it == pressures_kPa.begin()) {
        idx = 0;
    } else if (it == pressures_kPa.end()) {
        idx = pressures_kPa.size() - 2;
    } else {
        idx = std::distance(pressures_kPa.begin(), it) - 1;
    }

    double p0 = pressures_kPa[idx];
    double p1 = pressures_kPa[idx+1];
    double v0 = values[idx];
    double v1 = values[idx+1];
    double result = v0 + (p - p0) * (v1 - v0) / (p1 - p0);

    if (result <= 0) {
        result = 0;
    }

    return result;
}

