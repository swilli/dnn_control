#ifndef FULLSTATECONTROLLER_H
#define FULLSTATECONTROLLER_H

#include "controller.h"
#include "feedforwardneuralnetwork.h"

class ControllerFullState : public Controller {
	/*
	* This class represents a PD controller and generates the thrust for hovering over a specific target position with respect to the rotating asteroid reference frame. 
	* The sensor data input is assumed to be relative target state offset.
	* The PD controller is implemented using a FFNN with no hidden layer and a linear activation function.
    */
public:
    // The number of input dimensions the controller works with
    static const unsigned int kDimensions;

    ControllerFullState(const double &maximum_thrust);
    ControllerFullState(const double &maximum_thrust, const std::vector<double> &pd_coefficients);

    virtual ~ControllerFullState();


    // thrust = F(sensor_data), whereas F is a PD controller
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

    // Change the PD controller behaviour
    void SetCoefficients(const std::vector<double> &pd_coefficients);

private:
	// The behaviour, implemented using a FFNN
    FeedForwardNeuralNetwork neural_network_;
};

#endif // FULLSTATECONTROLLER_H
