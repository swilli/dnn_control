#ifndef CONTROLLERNEURALNETWORK_H
#define CONTROLLERNEURALNETWORK_H

#include "controller.h"
#include "feedforwardneuralnetwork.h"


class ControllerNeuralNetwork : public Controller {
	/*
	* This class represents a Neural Network controller and generates the thrust for hovering over a specific target position, or keeping a certain height (with respect to the rotating asteroid reference frame). 
	* The sensor data input is assumed to be either relative target state offset or optical flow and accelerometer data.
	* The Neural Network controller is implemented using a FFNN with one hidden layer and a sigmoid activation function.
    */
public:
    // The number of input dimensions the controller works with
	static const unsigned int kDimensions;

    ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden);
    ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights);

    virtual ~ControllerNeuralNetwork();

    // thrust = F(sensor_data), whereas F is a FFNN
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

    // Change the controller's behaviour by changing the NN's weights
    void SetWeights(const std::vector<double> &weights);

private:
	// The behaviour, implemented using a FFNN 
    FeedForwardNeuralNetwork neural_network_;
};

#endif // CONTROLLERNEURALNETWORK_H
