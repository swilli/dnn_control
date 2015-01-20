#ifndef FIXEDSIMULATION_H
#define FIXEDSIMULATION_H

#include "simulation.h"

class FixedSimulation : public Simulation {
public:
    FixedSimulation(const unsigned int &random_seed);
    FixedSimulation(const FixedSimulation &other);

    virtual ~FixedSimulation();

    FixedSimulation& operator=(const FixedSimulation &other);

    virtual boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > Evaluate();

    double FixedStepSize() const;

private:
    double fixed_step_size_;
};
#endif // FIXEDSIMULATION_H
