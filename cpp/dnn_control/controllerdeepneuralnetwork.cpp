#include "controllerdeepneuralnetwork.h"
#include "configuration.h"

StackedAutoencoder ControllerDeepNeuralNetwork::stacked_autoencoder_ = StackedAutoencoder(PATH_TO_AUTOENCODER_LAYER_CONFIGURATION);

ControllerDeepNeuralNetwork::ControllerDeepNeuralNetwork(const unsigned int &input_dimensions, const double &maximum_thrust, const unsigned int &num_hidden)
    : Controller(input_dimensions, maximum_thrust), neural_network_(stacked_autoencoder_.OutputDimension(), true, 3, NeuralNetwork::ActivationFunctionType::Linear, {{num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid}}) {
    number_of_parameters_ = neural_network_.Size();
    state_action_history_ = boost::circular_buffer<double>(stacked_autoencoder_.InputDimension());

    // Means: [0.00337522514158, -0.00884266311072, -0.0377062227285]
    //Stdevs: [0.462748207442, 0.412742800578, 0.391742657706]
    back_transformations_ = {
        {0.00337522514158, 0.462748207442},
        {-0.00884266311072, 0.412742800578},
        {-0.0377062227285, 0.391742657706}
    };
}

ControllerDeepNeuralNetwork::ControllerDeepNeuralNetwork(const unsigned int &input_dimensions, const double &maximum_thrust, const unsigned int &num_hidden, const std::vector<double> &weights)
    : Controller(input_dimensions, maximum_thrust), neural_network_(stacked_autoencoder_.OutputDimension(), true, 3, NeuralNetwork::ActivationFunctionType::Linear, {{num_hidden, true, NeuralNetwork::ActivationFunctionType::Sigmoid}}) {
    number_of_parameters_ = neural_network_.Size();
    state_action_history_ = boost::circular_buffer<double>(stacked_autoencoder_.InputDimension());
    SetWeights(weights);

    // Means: [0.00337522514158, -0.00884266311072, -0.0377062227285]
    //Stdevs: [0.462748207442, 0.412742800578, 0.391742657706]
    back_transformations_ = {
        {0.00337522514158, 0.462748207442},
        {-0.00884266311072, 0.412742800578},
        {-0.0377062227285, 0.391742657706}
    };
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

    if (state_action_history_.size() < stacked_autoencoder_.InputDimension()) {
        unboxed_thrust = {0.0, 0.0, 0.0};
    } else {
        const std::vector<double> state_actions(state_action_history_.begin(), state_action_history_.end());
        std::vector<double> predicted_velocity = stacked_autoencoder_.Evaluate(state_actions);

        for (unsigned int i = 0; i < predicted_velocity.size(); ++i) {
            predicted_velocity[i] = (predicted_velocity[i] - 0.5) * 4.0 * back_transformations_.at(i).second + back_transformations_.at(i).first;
        }
        unboxed_thrust = neural_network_.Evaluate(predicted_velocity);
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

