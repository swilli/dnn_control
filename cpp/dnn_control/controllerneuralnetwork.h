#ifndef CONTROLLERNEURALNETWORK_H
#define CONTROLLERNEURALNETWORK_H

#include "controller.h"
#include "feedforwardneuralnetwork.h"

#define CONTROLLER_DIMENSION    7
#define CONTROLLER_HIDDEN_DIMENSION 10

class ControllerNeuralNetwork : public Controller {
public:
    ControllerNeuralNetwork(const double &maximum_thrust);
    virtual ~ControllerNeuralNetwork();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

private:
    FeedForwardNeuralNetwork neural_network_;
};

#endif // CONTROLLERNEURALNETWORK_H
