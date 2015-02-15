#ifndef CONTROLLERNEURALNETWORK_H
#define CONTROLLERNEURALNETWORK_H

#include "controller.h"
#include "feedforwardneuralnetwork.h"


class ControllerNeuralNetwork : public Controller {
public:
    static const unsigned int kDimensions;

    ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden);
    ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights);

    virtual ~ControllerNeuralNetwork();

    // thrust = F(sensor_data), whereas F can be eg., a PID controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

    void SetWeights(const std::vector<double> &weights);

private:
    FeedForwardNeuralNetwork neural_network_;
};

#endif // CONTROLLERNEURALNETWORK_H
