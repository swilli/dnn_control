#ifndef CONTROLLERNEURALNETWORK_H
#define CONTROLLERNEURALNETWORK_H

#include "controller.h"
#include "configuration.h"

#if CNN_NEURAL_NETWORK_TYPE == CNN_NN_TYPE_FFNN
#include "feedforwardneuralnetwork.h"
#elif CNN_NEURAL_NETWORK_TYPE == CNN_NN_TYPE_ESRN
#include "simplerecurrentneuralnetwork.h"
#endif

class ControllerNeuralNetwork : public Controller {
public:
    static const unsigned int kDimensions;

    ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden);
    ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights);

    virtual ~ControllerNeuralNetwork();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

    void SetWeights(const std::vector<double> &weights);

private:

#if CNN_NEURAL_NETWORK_TYPE == CNN_NN_TYPE_FFNN
    FeedForwardNeuralNetwork neural_network_;
#elif CNN_NEURAL_NETWORK_TYPE == CNN_NN_TYPE_ESRN
    SimpleRecurrentNeuralNetwork neural_network_;
#endif
};

#endif // CONTROLLERNEURALNETWORK_H
