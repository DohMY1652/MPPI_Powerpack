#ifndef MPPI_H
#define MPPI_H

#include "mppi_brl/ControllerDatabaseConfig.h"


class MPPI {
    public:
        MPPI(std::shared_ptr<ControllerDatabaseConfig>& databaseconfig);
        ~MPPI();

    private:

};

#endif // MPPI_H