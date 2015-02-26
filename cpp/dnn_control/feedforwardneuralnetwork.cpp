#include "feedforwardneuralnetwork.h"
#include <cmath>

FeedForwardNeuralNetwork::FeedForwardNeuralNetwork(const unsigned int &dimension_input_layer, const bool &input_layer_enable_bias, const std::vector<boost::tuple<unsigned int, bool, ActivationFunctionType> > &layer_configurations)
    : NeuralNetwork(), dimension_input_layer_(dimension_input_layer), input_layer_enable_bias_(input_layer_enable_bias), layer_configurations_(layer_configurations) {

    unsigned int total_size = 0;

    unsigned int layer_size = 0;
    unsigned int bias = input_layer_enable_bias_;

    layer_size = (dimension_input_layer_ + bias) * boost::get<0>(layer_configurations_.at(0));

    layer_weights_.push_back(std::vector<double>(layer_size));
    total_size += layer_size;

    for (unsigned int i = 1; i < layer_configurations_.size(); ++i) {
        layer_size = 0;
        bias = boost::get<1>(layer_configurations_.at(i-1));
        layer_size = (boost::get<0>(layer_configurations_.at(i-1)) + bias) * boost::get<0>(layer_configurations_.at(i));
        layer_weights_.push_back(std::vector<double>(layer_size));
        total_size += layer_size;
    }

    size_ = total_size;
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
    unsigned int bias = input_layer_enable_bias_;
    unsigned int dim_input = dimension_input_layer_;

    for (unsigned int layer_index = 0; layer_index < layer_configurations_.size(); layer_index++) {
        const boost::tuple<unsigned int, bool, ActivationFunctionType> &next_layer_conf = layer_configurations_.at(layer_index);
        const std::vector<double> &layer_weights = layer_weights_.at(layer_index);

        const unsigned int dim_output = boost::get<0>(next_layer_conf);
        const ActivationFunctionType function_type = boost::get<2>(next_layer_conf);
        std::vector<double> layer_output(dim_output, 0.0);

        for (unsigned int i = 0; i < dim_output; ++i) {

            // Add bias
            if (bias == 1) {
                layer_output[i] = layer_weights[i * (dim_input + bias)];
            }

            // Add weighted input
            for (unsigned int j = 0; j < dim_input; ++j) {
                const unsigned int ji = i * (dim_input + bias) + j + bias;
                layer_output[i] += layer_weights[ji] * layer_input[j];
            }

            // Activation function
            switch (function_type) {
            case ActivationFunctionType::Linear:
                // do nothing
                break;
            case ActivationFunctionType::Sigmoid:
                layer_output[i] = 1.0 / (1.0 + std::exp(-layer_output[i]));
                break;
            }
        }

        layer_input = layer_output;
        dim_input = dim_output;
        bias = boost::get<1>(next_layer_conf);
    }

    return layer_input;
}

