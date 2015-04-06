#ifndef CONTROLLERDEEPNEURALNETWORK_H
#define CONTROLLERDEEPNEURALNETWORK_H

#include "controller.h"
#include "simplerecurrentneuralnetwork.h"
#include "stackedautoencoder.h"

#include <boost/circular_buffer.hpp>

class ControllerDeepNeuralNetwork : public Controller {
    /*
    * This class represents a deep Neural Network controller and generates the thrust for hovering over a specific target position, or keeping a certain height (with respect to the rotating asteroid reference frame).
    * The sensor data input is assumed to be either relative target state offset or optical flow and accelerometer data.
    * The Neural Network controller is implemented using a FFNN with one hidden layer and a sigmoid activation function.
    * In contrast to the standard neural network controller, this controller performs sensory compression and acts on the compressed sensor information.
    */
public:
    ControllerDeepNeuralNetwork(const unsigned int &input_dimensions, const double &maximum_thrust, const unsigned int &num_hidden);
    ControllerDeepNeuralNetwork(const unsigned int &input_dimensions, const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights);

    // thrust = F(sensor_data), whereas F is a FFNN
    virtual Vector3D GetThrustForSensorData(const std::vector<double> &sensor_data);

    // Change the controller's behaviour by changing the NN's weights
    void SetWeights(const std::vector<double> &weights);


private:
    // The compression, linearization layer
    static StackedAutoencoder stacked_autoencoder_;

    // The behaviour, implemented using a FFNN
    FeedForwardNeuralNetwork neural_network_;

    boost::circular_buffer<double> state_action_history_;

    std::vector<std::pair<double, double> > back_transformations_;
};

#endif // CONTROLLERDEEPNEURALNETWORK_H
