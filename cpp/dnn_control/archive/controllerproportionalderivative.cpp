#include "controllerproportionalderivative.h"
#include "configuration.h"

ControllerProportionalDerivative::ControllerProportionalDerivative(const unsigned int &input_dimensions, const double &maximum_thrust)
    : Controller(input_dimensions, maximum_thrust), neural_network_(input_dimensions, false, 3, NeuralNetwork::ActivationFunctionType::Linear, {}) {
    number_of_parameters_ = neural_network_.Size();
}

ControllerProportionalDerivative::ControllerProportionalDerivative(const unsigned int &input_dimensions, const double &maximum_thrust, const std::vector<double> &pd_coefficients)
    : Controller(input_dimensions, maximum_thrust), neural_network_(input_dimensions, false, 3, NeuralNetwork::ActivationFunctionType::Linear, {}) {
    number_of_parameters_ = neural_network_.Size();
    SetCoefficients(pd_coefficients);
}

void ControllerProportionalDerivative::SetCoefficients(const std::vector<double> &pd_coefficients) {
    if (pd_coefficients.size() == number_of_parameters_) {
        neural_network_.SetWeights(pd_coefficients);
    } else {
        throw SizeMismatchException();
    }
}

Vector3D ControllerProportionalDerivative::GetThrustForSensorData(const std::vector<double> &sensor_data) {
    const std::vector<double> unboxed_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        double t = unboxed_thrust[i];
        if (t > maximum_thrust_) {
            t = maximum_thrust_;
        } else if (t < -maximum_thrust_) {
            t = -maximum_thrust_;
        }
        thrust[i] = t;
    }
    return thrust;
}
