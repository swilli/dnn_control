#ifndef PAGMOSIMULATIONNEURALNETWORK_H
#define PAGMOSIMULATIONNEURALNETWORK_H

#include "pagmosimulation.h"

class PaGMOSimulationNeuralNetwork : public PaGMOSimulation {
public:
    const static unsigned int kHiddenNodes = 10;

    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const unsigned int &hidden_nodes);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const unsigned int &hidden_nodes, const std::vector<double> &neural_network_weights);

    virtual ~PaGMOSimulationNeuralNetwork();


    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateAdaptive();

    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateFixed();

private:
    unsigned int neural_network_hidden_nodes_;
};

#endif // PAGMOSIMULATIONNEURALNETWORK_H
