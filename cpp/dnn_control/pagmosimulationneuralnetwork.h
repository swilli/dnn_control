#ifndef PAGMOSIMULATIONNEURALNETWORK_H
#define PAGMOSIMULATIONNEURALNETWORK_H

#include "vector.h"
#include "samplefactory.h"
#include "asteroid.h"
#include "odesystem.h"
#include "sensorsimulatorneuralnetwork.h"
#include "controllerneuralnetwork.h"

#include <boost/tuple/tuple.hpp>

class PaGMOSimulationNeuralNetwork
{
public:
    PaGMOSimulationNeuralNetwork(const PaGMOSimulationNeuralNetwork &other);

    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const unsigned int &hidden_nodes);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights, const unsigned int &hidden_nodes);

    virtual ~PaGMOSimulationNeuralNetwork();

    PaGMOSimulationNeuralNetwork& operator=(const PaGMOSimulationNeuralNetwork &other);

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > Evaluate();

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateDetailed();

    double FixedStepSize() const;
    double MinimumStepSize() const;

    unsigned int ControllerNeuralNetworkSize() const;

    Asteroid& AsteroidOfSystem();

    class Exception {};
    class SizeMismatchException : public Exception {};

private:
    void Init();

    unsigned int random_seed_;

    unsigned int hidden_nodes_;

    double simulation_time_;

    double minimum_step_size_;

    double fixed_step_size_;

    double engine_noise_;

    double perturbation_noise_;

    double spacecraft_specific_impulse_;

    SampleFactory sample_factory_;

    Asteroid asteroid_;

    SensorSimulatorNeuralNetwork *sensor_simulator_;

    ControllerNeuralNetwork *controller_;

    SystemState initial_system_state_;
};

#endif // PAGMOSIMULATIONNEURALNETWORK_H
