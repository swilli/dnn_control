#include "controllerneuralnetwork.h"

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust) : Controller(CONTROLLER_DIMENSION, maximum_thrust), neural_network_(CONTROLLER_DIMENSION, CONTROLLER_HIDDEN_DIMENSION, 3) {

}

ControllerNeuralNetwork::~ControllerNeuralNetwork() {

}

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> normalized_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust[i] = (normalized_thrust[i] - 0.5) * (maximum_thrust_ * 2.0);
    }
    return thrust;
}
