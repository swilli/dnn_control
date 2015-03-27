#include "controllerdeepneuralnetwork.h"
#include "stackedautoencoder.h"
#include "configuration.h"

#if PGMOS_ENABLE_ODOMETRY
const unsigned int ControllerDeepNeuralNetwork::kDimensions = 6;
#else
const unsigned int ControllerDeepNeuralNetwork::kDimensions = PGMOS_ENABLE_OPTICAL_FLOW * 6 + PGMOS_ENABLE_VELOCITY * 3 + PGMOS_ENABLE_VELOCITY_OVER_HEIGHT * 3 + PGMOS_ENABLE_DIRECTION_SENSOR * 3 + PGMOS_ENABLE_ACCELEROMETER * 3;
#endif

static StackedAutoencoder kStackedAutoencoder(PATH_TO_AUTOENCODER_LAYER_CONFIGURATION);

ControllerDeepNeuralNetwork::ControllerDeepNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(kDimensions, maximum_thrust), neural_network_(kStackedAutoencoder.OutputSize(), true, {boost::make_tuple(num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid),
                                                               boost::make_tuple(3, false, NeuralNetwork::ActivationFunctionType::Linear)}) {
    number_of_parameters_ = neural_network_.Size();
    state_action_history_ = boost::circular_buffer<double>(kStackedAutoencoder.InputSize());
}

ControllerDeepNeuralNetwork::ControllerDeepNeuralNetwork(const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights)
    : Controller(kDimensions, maximum_thrust), neural_network_(kStackedAutoencoder.OutputSize(), true, {boost::make_tuple(num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid),
                                                               boost::make_tuple(3, false, NeuralNetwork::ActivationFunctionType::Linear)}) {
    number_of_parameters_ = neural_network_.Size();
    SetWeights(weights);
}

void ControllerDeepNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    if (weights.size() == number_of_parameters_) {
        neural_network_.SetWeights(weights);
    } else {
        throw SizeMismatchException();
    }
}


Vector3D ControllerDeepNeuralNetwork::GetThrustForSensorData(const std::vector<double> &sensor_data) {
    std::vector<double> unboxed_thrust;

    if (state_action_history_.size() < kStackedAutoencoder.InputSize()) {
        unboxed_thrust = {0.0, 0.0, 0.0};
    } else {
        const std::vector<double> state_actions(state_action_history_.begin(), state_action_history_.end());
        std::vector<double> compressed_state = kStackedAutoencoder.Compress(state_actions);
        for (unsigned int i = 0; i < compressed_state.size(); ++i) {
            compressed_state[i] *= 10.0;
        }
        unboxed_thrust = neural_network_.Evaluate(compressed_state);
    }

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

    for (unsigned int i = 0; i < sensor_data.size(); ++i) {
        state_action_history_.push_back(sensor_data.at(i));
    }
    for (unsigned int i = 0; i < thrust.size(); ++i) {
        const double ranged_thrust = (thrust[i] / maximum_thrust_ * 0.5) + 0.5;
        state_action_history_.push_back(ranged_thrust);
    }

    return thrust;
}

