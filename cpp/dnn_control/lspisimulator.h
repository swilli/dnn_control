#ifndef LSPISIMULATOR_H
#define LSPISIMULATOR_H

#include "vector.h"

#include "systemstate.h"

// r, r_dot, omega
typedef boost::array<double, 9> LSPIState;

class LSPISimulator {
public:
    LSPISimulator();
    ~LSPISimulator();

    SystemState NextState(const SystemState &state, const double &time, const Vector3D &thrust);

};

#endif // LSPISIMULATOR_H
