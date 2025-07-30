#include "Controller/MPPI.h"


MPPI::MPPI(std::shared_ptr<DatabaseConfig>& databaseconfig, std::shared_ptr<ComponentGoverner>& model)
  : databaseconfig(databaseconfig), model(model), 
  dt(databaseconfig->get_dt()), N(databaseconfig->get_N()), K(databaseconfig->get_K()),
  tracking_gain(databaseconfig->get_tracking_gain()),
  X(static_cast<int>(model->get_state().size())), U(static_cast<int>(model->get_control().size())),
    sampled_trajectory(K, std::vector<std::vector<double>>(N, std::vector<double>(X, 0.0))),
    sampled_control(K, std::vector<std::vector<double>>( N, std::vector<double>(U, 0.0))),
    rd(), gen(rd()), dist_real(0.0, 1.0) 
    {
        weight.assign(K, std::vector<double>(N, 0.0));
        control.assign(U, 0.0);
        target.assign(X, 0.0);
    }

MPPI::~MPPI() {}



std::vector<double> MPPI::calculate_control(const std::vector<double>& _target, const std::vector<double>& state) {
    target = _target;
    weight.assign(K, std::vector<double>(N, 0.0));
    controls.assign(N, std::vector<double>(U, 0.0));
    sample(state);
    calculate_weight();
    calculate_control();
    // print3DVector(sampled_trajectory);
    // print3DVector(sampled_control);
    // print2DVector(weight);
    // print2DVector(controls);
    // print1DVector(control);

    model->update_pump_state(dt);
    return control;
}

// private

void MPPI::sample(const std::vector<double>& state) {
    for (int k = 0; k < K; ++k) {
        model->set_state(state);
        for (int n = 0; n < N; ++n) {
            for (int u =0; u < U; ++u) {
                sampled_control[k][n][u] = dist_real(gen);
            }
            model->calculate_next_state(sampled_control[k][n]);
            // model->calculate_next_state({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            sampled_trajectory[k][n] = model->get_state();
        }
    }

}

void MPPI::calculate_weight() {
    for(int k = 0; k < K; ++k) {
        for(int n = 0; n < N; ++n) {
            for (int x = 0; x < X; ++x) {
                weight[k][n] += (tracking_gain * std::pow((target[x] - sampled_trajectory[k][n][x]),2));
            }
        }
    }
    double numerator = 0;
    double denominator = 0;
    for(int n = 0; n < N; ++n) {
        for (int u = 0; u < U; ++u) {
            for(int k = 0; k < K; ++k) {
                numerator += std::exp(-weight[k][n]) * sampled_control[k][n][u];
                denominator += std::exp(-weight[k][n]);
            }
            // std::cout << numerator << ", " << denominator << std::endl;

            if (denominator <= 0.0000) {
                controls[n][u] = 0;
            } else {
                controls[n][u] = numerator/denominator;
            }
            numerator = 0;
            denominator = 0;
        }
    }
}

void MPPI::calculate_control() {
    for(int u = 0; u < U; ++u) {
        control[u] = controls[0][u];
    }
    control[0] = 1.0;
}

void MPPI::print3DVector(const std::vector<std::vector<std::vector<double>>>& vec) {
    size_t K = vec.size();
    if (K == 0) {
        std::cout << "Empty 3D vector\n";
        return;
    }
    size_t N = vec[0].size();
    size_t X = (N > 0 ? vec[0][0].size() : 0);

    std::cout << "Dimensions: K=" << K 
              << ", N=" << N 
              << ", X=" << X << "\n\n";

    for (size_t k = 0; k < K; ++k) {
        std::cout << "Slice k=" << k << ":\n";
        for (size_t n = 0; n < N; ++n) {
            std::cout << "  [ ";
            for (size_t x = 0; x < X; ++x) {
                std::cout << vec[k][n][x];
                if (x + 1 < X) std::cout << ", ";
            }
            std::cout << " ]\n";
        }
        std::cout << "\n";
    }
}

void MPPI::print2DVector(const std::vector<std::vector<double>>& vec) {
    size_t N = vec.size();
    if (N == 0) {
        std::cout << "Empty 2D vector\n";
        return;
    }
    size_t X = vec[0].size();

    std::cout << "Dimensions: N=" << N 
              << ", X=" << X << "\n\n";

    for (size_t n = 0; n < N; ++n) {
        std::cout << "[ ";
        for (size_t x = 0; x < X; ++x) {
            std::cout << vec[n][x];
            if (x + 1 < X) std::cout << ", ";
        }
        std::cout << " ]\n";
    }
}

void MPPI::print1DVector(const std::vector<double>& vec) {
    size_t X = vec.size();
    if (X == 0) {
        std::cout << "Empty 1D vector\n";
        return;
    }

    std::cout << "Dimensions: X=" << X << "\n\n";

    std::cout << "[ ";
    for (size_t x = 0; x < X; ++x) {
        std::cout << vec[x];
        if (x + 1 < X) std::cout << ", ";
    }
    std::cout << " ]\n";
}
