#include "controllerneuralnetwork.h"
#include "constants.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerNeuralNetwork::kDimensions = 6;
#else
const unsigned int ControllerNeuralNetwork::kDimensions = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3 + PGMOS_ENABLE_ACCELEROMETER * 3;
#endif

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(kDimensions, maximum_thrust), neural_network_(kDimensions, true, {boost::make_tuple(num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid),
                                                               boost::make_tuple(3, false, NeuralNetwork::ActivationFunctionType::Sigmoid)}) {
    number_of_parameters_ = neural_network_.Size();
}

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights)
    : Controller(kDimensions, maximum_thrust), neural_network_(kDimensions, true, {boost::make_tuple(num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid),
                                                               boost::make_tuple(3, false, NeuralNetwork::ActivationFunctionType::Sigmoid)}) {
    number_of_parameters_ = neural_network_.Size();
    SetWeights(weights);
}

void ControllerNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    if (weights.size() == number_of_parameters_) {
        neural_network_.SetWeights(weights);
    } else {
        throw SizeMismatchException();
    }
}

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const SensorData &sensor_data) {
#if CNN_ENABLE_CORRECT_THRUST_OUTPUT
    const std::vector<double> normalized_u_v_t = neural_network_.Evaluate(sensor_data);
    const double u = 2.0 * kPi * normalized_u_v_t[0];
    const double v = acos(2.0 * normalized_u_v_t[1] - 1.0);
    const double t = normalized_u_v_t[2] * maximum_thrust_;

    Vector3D thrust;
    thrust[0] = cos(u) * sin(v) * t;
    thrust[1] = sin(u) * sin(v) * t;
    thrust[2] = cos(v) * t;
    return thrust;
#else
    const std::vector<double> unscaled_thrust = neural_network_.Evaluate(sensor_data);
    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust[i] = (unscaled_thrust[i] - 0.5) * maximum_thrust_;
    }
    return thrust;
#endif
}
