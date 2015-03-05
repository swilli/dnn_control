#ifndef PAGMOSIMULATIONPROPORTIONALDERIVATIVE_H
#define PAGMOSIMULATIONPROPORTIONALDERIVATIVE_H

#include "pagmosimulation.h"

class PaGMOSimulationProportionalDerivative : public PaGMOSimulation {
	/*
    * This class represents a full simulation with a PD controller.
    */
public:
    PaGMOSimulationProportionalDerivative(const unsigned int &random_seed);
    PaGMOSimulationProportionalDerivative(const unsigned int &random_seed, const std::vector<double> &pd_coefficients);


    // Simulates the configured simulation, used an adaptive integrator
    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateAdaptive();

    // Simulates the configured simulation, used a fixed integrator
    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateFixed();

    // Returns the number of parameters the controller has. 
    virtual unsigned int ChromosomeSize() const;
};

#endif // PAGMOSIMULATIONPROPORTIONALDERIVATIVE_H
