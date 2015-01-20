#ifndef PAGMOSIMULATION_H
#define PAGMOSIMULATION_H

#include "vector.h"
#include "samplefactory.h"
#include "asteroid.h"
#include "odesystem.h"
#include "sensorsimulatorneuralnetwork.h"
#include "controllerneuralnetwork.h"

#include <boost/tuple/tuple.hpp>

class PaGMOSimulation
{
public:
    PaGMOSimulation();
    PaGMOSimulation(const PaGMOSimulation &other);

    PaGMOSimulation(const unsigned int &random_seed);
    PaGMOSimulation(const unsigned int &random_seed, const std::vector<double> &neural_network_weights);

    virtual ~PaGMOSimulation();

    PaGMOSimulation& operator=(const PaGMOSimulation &other);

    boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > Evaluate();

    boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<double> > EvaluateDetailed();

    double FixedStepSize() const;
    double MinimumStepSize() const;

    Asteroid& AsteroidOfSystem();

private:
    void Init();

    unsigned int random_seed_;

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

#endif // PAGMOSIMULATION_H
