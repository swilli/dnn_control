#ifndef FULLSTATECONTROLLER_H
#define FULLSTATECONTROLLER_H

#include "controller.h"
#include "feedforwardneuralnetwork.h"

class ControllerFullState : public Controller {
public:
    static const unsigned int kDimensions;

    ControllerFullState(const double &maximum_thrust);
    ControllerFullState(const double &maximum_thrust, const std::vector<double> &pd_coefficients);

    virtual ~ControllerFullState();


    // thrust = F(sensor_data), whereas F can be eg., a PID controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

    void SetCoefficients(const std::vector<double> &pd_coefficients);

private:
    FeedForwardNeuralNetwork neural_network_;
};

#endif // FULLSTATECONTROLLER_H
