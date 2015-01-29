#ifndef PAGMOSIMULATIONNEURALNETWORK_H
#define PAGMOSIMULATIONNEURALNETWORK_H

#include "vector.h"
#include "asteroid.h"
#include "systemstate.h"

#include <boost/tuple/tuple.hpp>

#define PSNN_TEST_FOR_ORBIT     0

class PaGMOSimulationNeuralNetwork {
public:
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const unsigned int &hidden_nodes);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &neural_network_weights);
    PaGMOSimulationNeuralNetwork(const unsigned int &random_seed, const double &simulation_time, const unsigned int &hidden_nodes, const std::vector<double> &neural_network_weights);

    virtual ~PaGMOSimulationNeuralNetwork();


    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateAdaptive();

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateFixed();


    double FixedStepSize() const;

    double MinimumStepSize() const;

    double InteractionInterval() const;

    unsigned int ControllerNeuralNetworkSize() const;

    Asteroid& AsteroidOfSystem();

    class Exception {};
    class SizeMismatchException : public Exception {};

private:
    class Observer {
    public:
        Observer(double &time) : time_(time){}
        void operator () (const SystemState &, const double &current_time) {
            time_ = current_time;
        }
    private:
        double &time_;
    };

    void Init();

    unsigned int random_seed_;

    unsigned int neural_network_hidden_nodes_;

    double simulation_time_;

    double interaction_interval_;

    double minimum_step_size_;

    double fixed_step_size_;

    double spacecraft_engine_noise_;

    double spacecraft_specific_impulse_;

    double spacecraft_mass_;

    double spacecraft_minimum_mass_;

    double spacecraft_maximum_thrust_;

    double perturbation_mean_;

    double perturbation_noise_;

    Asteroid asteroid_;

    SystemState initial_system_state_;

    Vector3D target_position_;

    std::vector<double> neural_network_weights_;
};

#endif // PAGMOSIMULATIONNEURALNETWORK_H
