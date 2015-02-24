#include "controllerneuralnetwork.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerNeuralNetwork::kDimensions = 6;
#else
#if PGMOS_ENABLE_ACCELEROMETER
const unsigned int ControllerNeuralNetwork::kDimensions = 7;
#else
const unsigned int ControllerNeuralNetwork::kDimensions = 4;
#endif
#endif

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, true), std::make_pair(num_hidden, true), std::make_pair(4, false)}, NeuralNetwork::ActivationFunctionType::Sigmoid) {
    number_of_parameters_ = neural_network_.Size();
}

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, true), std::make_pair(num_hidden, true), std::make_pair(4, false)}, NeuralNetwork::ActivationFunctionType::Sigmoid) {
    number_of_parameters_ = neural_network_.Size();
    SetWeights(weights);
}

ControllerNeuralNetwork::~ControllerNeuralNetwork() {

}

void ControllerNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    if (weights.size() == number_of_parameters_) {
        neural_network_.SetWeights(weights);
    } else {
        throw SizeMismatchException();
    }
}

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> direction_magnitude = neural_network_.Evaluate(sensor_data);
    Vector3D direction = {direction_magnitude[0], direction_magnitude[1], direction_magnitude[2]};
    const double magnitude = (2.0 * direction_magnitude[3] - 1.0) * maximum_thrust_;
    const double norm_direction = VectorNorm(direction);
    if (norm_direction) {
        const double coef_norm= 1.0 / norm_direction;
        direction[0] *= coef_norm;
        direction[1] *= coef_norm;
        direction[2] *= coef_norm;
    } else {
        return {0.0, 0.0, 0.0};
    }
    Vector3D thrust(direction);
    thrust[0] *= magnitude;
    thrust[1] *= magnitude;
    thrust[2] *= magnitude;
    return thrust;
}
