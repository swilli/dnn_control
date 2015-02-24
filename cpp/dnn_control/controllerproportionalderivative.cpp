#include "controllerproportionalderivative.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerProportionalDerivative::kDimensions = 6;
#else
#if PGMOS_ENABLE_ACCELEROMETER
const unsigned int ControllerProportionalDerivative::kDimensions = 7;
#else
const unsigned int ControllerProportionalDerivative::kDimensions = 3;
#endif
#endif

ControllerProportionalDerivative::ControllerProportionalDerivative(const double &maximum_thrust)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, false), std::make_pair(4, false)}, NeuralNetwork::ActivationFunctionType::Linear) {
    number_of_parameters_ = neural_network_.Size();
}

ControllerProportionalDerivative::ControllerProportionalDerivative(const double &maximum_thrust, const std::vector<double> &pd_coefficients)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, false), std::make_pair(4, false)}, NeuralNetwork::ActivationFunctionType::Linear) {
    number_of_parameters_ = neural_network_.Size();
    SetCoefficients(pd_coefficients);
}

ControllerProportionalDerivative::~ControllerProportionalDerivative() {

}

void ControllerProportionalDerivative::SetCoefficients(const std::vector<double> &pd_coefficients) {
    if (pd_coefficients.size() == number_of_parameters_) {
        neural_network_.SetWeights(pd_coefficients);
    } else {
        throw SizeMismatchException();
    }
}

Vector3D ControllerProportionalDerivative::GetThrustForSensorData(const SensorData &sensor_data) {
    const std::vector<double> direction_magnitude = neural_network_.Evaluate(sensor_data);
    Vector3D direction = {direction_magnitude[0], direction_magnitude[1], direction_magnitude[2]};
    double magnitude = direction_magnitude[3] * maximum_thrust_;
    if (magnitude > maximum_thrust_) {
        magnitude = maximum_thrust_;
    } else if (magnitude < -maximum_thrust_) {
        magnitude = -maximum_thrust_;
    }
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
