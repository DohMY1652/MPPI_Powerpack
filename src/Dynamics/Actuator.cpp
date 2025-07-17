#include "mppi_brl/Dynamics/Actuator.h"

Actuator::Actuator(const std::vector<double>& system_parameters)
    : ControlTargetComponent(system_parameters)  {
        D = system_parameters[0] *0.001; // m
        r = system_parameters[1] *0.001; // m
        external_load = system_parameters[2];  // kg
        L_max = system_parameters[3] *0.001; //m
        dt = system_parameters[4];  // sec
        positive_D = system_parameters[5] *0.001; // m
        positive_L = system_parameters[6] *0.001; // m
        V_tot = M_PI * (D / (2 * sin(M_PI/8))) * (D / (2 * sin(M_PI/8))) * L_max * 2;  // m^3
        build_lookup_table();

    }




   double Actuator::get_force() const {
    return F_tot;
   } 



double Actuator::calculate_pressure_dot(double flow_rate_in, double flow_rate_out) {
    return 0;
}

double Actuator::calculate_positive_pressure_dot(double flow_rate_in, double flow_rate_out) {
    return (((flow_rate_in - flow_rate_out)*R*T)-(P_pos * V_pos_dot)) / V_pos;
}

double Actuator::calculate_negative_pressure_dot(double flow_rate_in, double flow_rate_out) {
    return (((flow_rate_in - flow_rate_out)*R*T)-(P_neg * V_neg_dot)) / V_neg;
}

void Actuator::calculate_positive_and_negative_volume(double x, double x_dot) {
    double L = L_max - x;
    double L_dot = -1 * x_dot;
    if ((1-L/L_max)<= 0.3634) {
        if (theta <= 0.0001) {
            theta_dot = - 3 * (L_max * theta) * L_dot;
        } else {
            theta_dot = (theta * theta) / (L_max * (theta * cos(theta) - sin (theta))) * L_dot;
        }
        V_neg_dot = (4*D*L_max*L_max/(theta*theta*theta)) * (theta*std::cos(theta)*std::cos(theta) - std::sin(theta)*std::cos(theta)) * theta_dot;
        V_neg = 2*D*D*L*(1+std::sqrt(2.0)) - (2*D*L_max*L_max/theta)*(1 - std::sin(theta)*std::cos(theta)/theta);;
    } else {
        theta_dot = 0;
        V_neg = 2 * D * D * (1 + sqrt(2)) * L + M_PI * D * L * L - 4 * L_max * L * D;
        V_neg_dot = (2 * D * D * (1 + sqrt(2)) + 2 * M_PI * D * L - 4 * L_max * D) * L_dot;
    }
    V_pos = V_tot - V_neg - r * r * x;
    V_pos_dot = -1 * V_neg_dot - r * r * x_dot; 
    
}


double Actuator::calculate_length_dot(double positive_pressure, double negative_pressure, double x) {
    double P_pos  = (positive_pressure - 101.325) * 1000.0;  // Pa
    double P_neg  = (negative_pressure - 101.325) * 1000.0;  // Pa
    double P_diff = P_pos - P_neg;                          // Pa

    double L = L_max - x;

    if ((1.0 - L / L_max) <= 0.3634) {
        theta = lookup_theta(x);
        F_tot = P_diff * (2*D*D*(1+std::sqrt(2.0)) - M_PI*r*r - (2*D*L_max/(theta*std::cos(theta)-std::sin(theta)))*(1 + (theta*std::cos(2*theta)-std::sin(2*theta))/theta));
    } else {
        F_tot = (P_pos - P_neg) * (2*D*D*(1 + std::sqrt(2.0)) + (2*D*L_max/(theta*std::cos(theta) - std::sin(theta))) * (1 + (theta*std::cos(2*theta) - std::sin(2*theta)) / theta)) - P_pos * M_PI * r * r;
    }

    double x_ddot = F_tot / external_load - g;
    if ((x >= 0.0175 && x_ddot >= 0.0) ||
        (x <= 0.0  && x_ddot <= 0.0)) {
        x_ddot = 0.0;
        x_dot = 0.0;
    } else {
        x_dot += x_ddot * dt;
    }

    calculate_positive_and_negative_volume(x, x_dot);

    return x_dot;

}

void Actuator::set_positive_pressure(double pressure){
    P_pos = pressure;
}

void Actuator::set_negative_pressure(double pressure){
    P_neg = pressure;
}



std::vector<double> Actuator::get_L_table() const {
    return l_table;
}

std::vector<double> Actuator::get_theta_table() const {
    return theta_table;
}


// private


void Actuator::build_lookup_table() {
    int N = static_cast<int>((l_max - l_min) / del_l);
    l_table.resize(N+1);
    theta_table.resize(N+1);
    for (int i = 0; i <= N; ++i) {
        double li = l_min + i * del_l;
        l_table[i] = li;
        theta_table[i] = solve_theta(l_0, li);

    }
}


double Actuator::f(double theta, double l0, double l) {
    return (l0 * std::sin(theta) / theta) - l;
}

double Actuator::df(double theta, double l0) {
    return l0 * (theta * std::cos(theta) - std::sin(theta)) / (theta * theta);
}

double Actuator::solve_theta(double l0, double l) {
    static constexpr double initial_guesses[] = {
        0.5, 1.0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI
    };
    for (double theta0 : initial_guesses) {
        double theta = theta0;
        bool converged = false;

        // 뉴턴-랩슨 반복
        for (int i = 0; i < maxIter; ++i) {
            double ft  = f(theta, l0, l);
            double dft = df(theta, l0);
            if (std::fabs(dft) < std::numeric_limits<double>::epsilon()) 
                break;
            double delta = ft / dft;
            theta -= delta;
            if (std::fabs(delta) < tol) {
                converged = true;
                break;
            }
        }
        if (converged) {
            return theta;
        }
        std::cerr << "Guess " << theta0 << " failed, ";
    }

    std::cerr<< "Error: all guesses failed. Returning NaN.\n";
    return std::numeric_limits<double>::quiet_NaN();
}

double Actuator::lookup_theta(double l){
    auto it = std::lower_bound(l_table.begin(), l_table.end(), l);
    if (it == l_table.begin())      return theta_table.front();
    if (it == l_table.end())        return theta_table.back();
    size_t idx = std::distance(l_table.begin(), it);
    double l1 = l_table[idx-1], l2 = l_table[idx];
    double t1 = theta_table[idx-1], t2 = theta_table[idx];
    double w = (l - l1) / (l2 - l1);

    return t1 + w * (t2 - t1);
}