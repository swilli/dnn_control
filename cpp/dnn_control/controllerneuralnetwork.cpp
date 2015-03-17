#include "controllerneuralnetwork.h"
#include "constants.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerNeuralNetwork::kDimensions = 6;
#else
const unsigned int ControllerNeuralNetwork::kDimensions = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3 + PGMOS_ENABLE_ACCELEROMETER * 3;
#endif

#if CNN_ENABLE_STACKED_AUTOENCODER
#include "stackedautoencoder.h"
static StackedAutoencoder kStackedAutoencoder(PATH_TO_AUTOENCODER_LAYER_CONFIGURATION);
#endif

ControllerNeuralNetwork::ControllerNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(kDimensions, maximum_thrust), neural_network_(kDimensions, true, {boost::make_tuple(num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid),
                                                               boost::make_tuple(3, false, NeuralNetwork::ActivationFunctionType::Sigmoid)}) {
    number_of_parameters_ = neural_network_.Size();

#if CNN_ENABLE_STACKED_AUTOENCODER
    state_action_history_ =  boost::circular_buffer<double>(CNN_STACKED_AUTOENCODER_DIMENSIONS);
#endif
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

Vector3D ControllerNeuralNetwork::GetThrustForSensorData(const std::vector<double> &sensor_data) {
#if CNN_ENABLE_STACKED_AUTOENCODER
    std::vector<double> unscaled_thrust;
    if (state_action_history_.size() < CNN_STACKED_AUTOENCODER_DIMENSIONS) {
        unscaled_thrust = {0.5, 0.5, 0.5};
    } else {
        const std::vector<double> state_actions(state_action_history_.begin(), state_action_history_.end());
        unscaled_thrust = neural_network_.Evaluate(kStackedAutoencoder.Compress(state_actions));
    }
#else
    const std::vector<double> unscaled_thrust = neural_network_.Evaluate(sensor_data);
#endif

    Vector3D thrust;
    for (unsigned int i = 0; i < 3; ++i) {
        thrust[i] = (unscaled_thrust[i] - 0.5) * maximum_thrust_;
    }

#if CNN_ENABLE_STACKED_AUTOENCODER
    state_action_history_.push_front(unscaled_thrust[2]);
    state_action_history_.push_front(unscaled_thrust[1]);
    state_action_history_.push_front(unscaled_thrust[0]);
    for (int i = sensor_data.size() - 1; i >= 0; --i) {
        state_action_history_.push_front(sensor_data.at(i));
    }
#endif
    return thrust;
}
