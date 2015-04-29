#ifndef PAGMOSIMULATIONNEURALNETWORK_H
#define PAGMOSIMULATIONNEURALNETWORK_H

#include "pagmosimulation.h"
#include "configuration.h"

class PaGMOSimulationNeuralNetwork : public PaGMOSimulation {
    /*
    * This class represents a full simulation with a neural network controller.
    */
public:
    // The number of default hidden nodes the controller neural network has
    const static unsigned int kHiddenNodes = 10;

    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const std::set<SensorSimulator::SensorType> &control_sensor_types={}, const bool &control_with_noise=false, const std::set<SensorSimulator::SensorType> &recording_sensor_types={}, const bool &recording_with_noise=false);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const unsigned int &hidden_nodes, const std::set<SensorSimulator::SensorType> &control_sensor_types={}, const bool &control_with_noise=false, const std::set<SensorSimulator::SensorType> &recording_sensor_types={}, const bool &recording_with_noise=false);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const std::vector<double> &neural_network_weights, const std::set<SensorSimulator::SensorType> &control_sensor_types={}, const bool &control_with_noise=false, const std::set<SensorSimulator::SensorType> &recording_sensor_types={}, const bool &recording_with_noise=false);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const unsigned int &hidden_nodes, const std::vector<double> &neural_network_weights, const std::set<SensorSimulator::SensorType> &control_sensor_types={}, const bool &control_with_noise=false, const std::set<SensorSimulator::SensorType> &recording_sensor_types={}, const bool &recording_with_noise=false);


    // Simulates the configured simulation, used an adaptive integrator
    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > EvaluateAdaptive();

    // Simulates the configured simulation, used a fixed integrator
    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<std::vector<double> > > EvaluateFixed();

    // Returns the number of parameters the controller has. 
    virtual unsigned int ChromosomeSize() const;

private:
    // The number of hidden nodes in the neural network controller
    unsigned int neural_network_hidden_nodes_;
};

#endif // PAGMOSIMULATIONNEURALNETWORK_H
