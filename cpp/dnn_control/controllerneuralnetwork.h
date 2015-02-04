#ifndef CONTROLLERNEURALNETWORK_H
#define CONTROLLERNEURALNETWORK_H

#define NN_TYPE_FFNN    0
#define NN_TYPE_ESRN    1
#define NN_TYPE_CTRNN   2

#define NEURAL_NETWORK_TYPE     NN_TYPE_FFNN

#include "controller.h"

#if NEURAL_NETWORK_TYPE == NN_TYPE_FFNN
#include "feedforwardneuralnetwork.h"
#elif NEURAL_NETWORK_TYPE == NN_TYPE_ESRN
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

    unsigned int NeuralNetworkSize() const;

private:

#if NEURAL_NETWORK_TYPE == NN_TYPE_FFNN
    FeedForwardNeuralNetwork neural_network_;
#elif NEURAL_NETWORK_TYPE == NN_TYPE_ESRN
    SimpleRecurrentNeuralNetwork neural_network_;
#endif
};

#endif // CONTROLLERNEURALNETWORK_H
