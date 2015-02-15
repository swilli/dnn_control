#include "controllerfullstate.h"

const unsigned int ControllerFullState::kDimensions = 6;

ControllerFullState::ControllerFullState(const double &maximum_thrust)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, false), std::make_pair(3 , false)}, NeuralNetwork::ActivationFunctionType::Linear) {        
    number_of_parameters_ = neural_network_.Size();
}

ControllerFullState::ControllerFullState(const double &maximum_thrust, const std::vector<double> &pd_coefficients)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, false), std::make_pair(3 , false)}, NeuralNetwork::ActivationFunctionType::Linear) {
    number_of_parameters_ = neural_network_.Size();
    SetCoefficients(pd_coefficients);
}

ControllerFullState::~ControllerFullState() {

}

void ControllerFullState::SetCoefficients(const std::vector<double> &pd_coefficients) {
    if (pd_coefficients.size() == number_of_parameters_) {
        neural_network_.SetWeights(pd_coefficients);
    } else {
        throw SizeMismatchException();
    }
}

Vector3D ControllerFullState::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> thrust = neural_network_.Evaluate(sensor_data);
    Vector3D normalized_thrust = {thrust[0], thrust[1], thrust[2]};
    normalized_thrust = VectorNormalize(normalized_thrust);
    for (unsigned int i = 0; i < 3; ++i) {
        normalized_thrust[i] *= maximum_thrust_;
    }
    return normalized_thrust;
}
