#include "controllerneuralnetwork.h"

#if CNN_WITH_VELOCITY
const unsigned int ControllerNeuralNetwork::kDimensions = 6;
#else
const unsigned int ControllerNeuralNetwork::kDimensions = 3;
#endif

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(kDimensions, maximum_thrust), neural_network_(dimensions_, num_hidden, 3) {

}

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights)
    : Controller(kDimensions, maximum_thrust), neural_network_(dimensions_, num_hidden, 3) {
    SetWeights(weights);
}

ControllerNeuralNetwork::~ControllerNeuralNetwork() {

}

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> normalized_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust[i] = (normalized_thrust[i] * maximum_thrust_ * 2.0) - maximum_thrust_;
    }
    return thrust;
}

void ControllerNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    try {
        neural_network_.SetWeights(weights);
    } catch (const NeuralNetwork::Exception &exception) {
        throw SizeMismatchException();
    }
}

unsigned int ControllerNeuralNetwork::NeuralNetworkSize() const {
    return neural_network_.Size();
}
