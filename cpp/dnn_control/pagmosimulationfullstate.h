#ifndef PAGMOSIMULATIONFULLSTATE_H
#define PAGMOSIMULATIONFULLSTATE_H

#include "pagmosimulation.h"

class PaGMOSimulationFullState : public PaGMOSimulation {
public:
    PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time);
    PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &pid_coefficients);

    virtual ~PaGMOSimulationFullState();


    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateAdaptive();

    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateFixed();

    virtual unsigned int ChromosomeSize() const;
};

#endif // PAGMOSIMULATIONFULLSTATE_H
