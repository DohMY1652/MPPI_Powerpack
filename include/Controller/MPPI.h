#ifndef MPPI_H
#define MPPI_H

#include "DatabaseConfig.h"
#include "Dynamics/ComponentGoverner.h"

#include <random>
#include <cmath>

class MPPI {
    public:
        MPPI(std::shared_ptr<DatabaseConfig>& databaseconfig,std::shared_ptr<ComponentGoverner>& model);
        ~MPPI();

        std::vector<double> calculate_control(const std::vector<double>& _target, const std::vector<double>& state);

    private:
        std::shared_ptr<DatabaseConfig> databaseconfig;
        std::shared_ptr<ComponentGoverner>    model; 

        double dt, tracking_gain;
        int N, K, X, U;
        std::vector<std::vector<std::vector<double>>> sampled_trajectory;
        std::vector<std::vector<std::vector<double>>> sampled_control;
        std::vector<std::vector<double>> weight;
        std::vector<std::vector<double>> controls;
        std::vector<double> control;

        std::vector<double> target;

        void sample(const std::vector<double>& state);
        void calculate_weight();
        void calculate_control();

        void print3DVector(const std::vector<std::vector<std::vector<double>>>& vec);
        void print2DVector(const std::vector<std::vector<double>>& vec);
        void print1DVector(const std::vector<double>& vec);

        std::random_device rd;
        std::mt19937 gen;
        std::uniform_real_distribution<double> dist_real;



};

#endif // MPPI_H