#include "feedforwardneuralnetwork.h"
#include <cmath>

FeedForwardNeuralNetwork::FeedForwardNeuralNetwork(const std::vector<std::pair<unsigned int, bool> > &layer_configurations, const ActivationFunctionType &function_type )
    : NeuralNetwork(), activation_function_type_(function_type), layer_configurations_(layer_configurations) {

    unsigned int total_size = 0;

    for (unsigned int i = 1; i < layer_configurations.size(); ++i) {
        unsigned int layer_size = 0;
        if (layer_configurations.at(i-1).second) {
            layer_size = (layer_configurations.at(i-1).first + 1) * layer_configurations.at(i).first;

        } else {
            layer_size = layer_configurations.at(i-1).first * layer_configurations.at(i).first;
        }
        layer_weights_.push_back(std::vector<double>(layer_size));
        total_size += layer_size;
    }

    size_ = total_size;
}

FeedForwardNeuralNetwork::~FeedForwardNeuralNetwork() {

}

void FeedForwardNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    if (size_ != weights.size()) {
        throw SizeMismatchException();
    }
    unsigned int layer_index = 0;
    unsigned int weight_index = 0;
    while (layer_index < layer_weights_.size()) {
        for (unsigned int i = 0; i < layer_weights_.at(layer_index).size(); ++i) {
            layer_weights_.at(layer_index).at(i) = weights.at(weight_index++);
        }
        layer_index++;
    }
}

std::vector<double> FeedForwardNeuralNetwork::Evaluate(const std::vector<double> &input) {

    std::vector<double> layer_input(input);

    for (unsigned int layer_index = 0; layer_index < layer_weights_.size(); layer_index++) {
        const std::pair<unsigned int, bool> &layer_conf = layer_configurations_.at(layer_index);
        const std::pair<unsigned int, bool> &next_layer_conf = layer_configurations_.at(layer_index + 1);
        const std::vector<double> &layer_weights = layer_weights_.at(layer_index);

        const unsigned int bias = layer_conf.second;
        const unsigned int dim_input = layer_conf.first + bias;
        const unsigned int dim_output = next_layer_conf.first;
        std::vector<double> layer_output(dim_output, 0.0);

        for (unsigned int i = 0; i < dim_output; ++i) {

            // Add bias
            if (layer_conf.second) {
                layer_output[i] = layer_weights[i * dim_input];
            }

            // Add weighted input
            for (unsigned int j = 0; j < layer_conf.first; ++j) {
                const unsigned int ji = i * dim_input + j + bias;
                layer_output[i] += layer_weights[ji] * layer_input[j];
            }

            // Activation function
            switch (activation_function_type_) {
            case ActivationFunctionType::Linear:
                // do nothing
                break;
            case ActivationFunctionType::Sigmoid:
                layer_output[i] = 1.0 / (1.0 + std::exp(-layer_output[i]));
                break;
            }
        }

        layer_input = layer_output;
    }

    return layer_input;
}

