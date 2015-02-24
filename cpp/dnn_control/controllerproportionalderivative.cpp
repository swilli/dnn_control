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
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, false), std::make_pair(3, false)}, NeuralNetwork::ActivationFunctionType::Linear) {
    number_of_parameters_ = neural_network_.Size();
}

ControllerProportionalDerivative::ControllerProportionalDerivative(const double &maximum_thrust, const std::vector<double> &pd_coefficients)
    : Controller(kDimensions, maximum_thrust), neural_network_({std::make_pair(kDimensions, false), std::make_pair(3, false)}, NeuralNetwork::ActivationFunctionType::Linear) {
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
    const std::vector<double> unscaled_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        double t = unscaled_thrust[i];
        if (t > maximum_thrust_) {
            t = maximum_thrust_;
        } else if (t < -maximum_thrust_) {
            t = -maximum_thrust_;
        }
        thrust[i] = t;
    }
    return thrust;
}
