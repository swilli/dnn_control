#ifndef ADAPTIVESIMULATION_H
#define ADAPTIVESIMULATION_H

#include "simulation.h"

class AdaptiveSimulation : public Simulation {
public:
    AdaptiveSimulation(const unsigned int &random_seed);
    AdaptiveSimulation(const AdaptiveSimulation &other);

    virtual ~AdaptiveSimulation();


    AdaptiveSimulation& operator=(const AdaptiveSimulation &other);

    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> >  Evaluate();

    double MinimumStepSize() const;

private:
    double minimum_step_size_;
};

#endif // ADAPTIVESIMULATION_H
