#include "feedforwardneuralnetwork.h"
#include <cmath>

FeedForwardNeuralNetwork::FeedForwardNeuralNetwork()
    : NeuralNetwork(), dimension_input_layer_(0), input_layer_enable_bias_(false), dimension_output_layer_(0) {

}

FeedForwardNeuralNetwork::FeedForwardNeuralNetwork(const unsigned int &dimension_input_layer, const bool &input_layer_enable_bias,
                                                   const unsigned int &dimension_output_layer, const ActivationFunctionType &output_layer_activation,
                                                   const std::vector<boost::tuple<unsigned int, bool, ActivationFunctionType> > &layer_configurations)
    : NeuralNetwork(), dimension_input_layer_(dimension_input_layer), input_layer_enable_bias_(input_layer_enable_bias),
      dimension_output_layer_(dimension_output_layer), output_layer_activation_(output_layer_activation), layer_configurations_(layer_configurations) {

    unsigned int total_size = 0;

    unsigned int dim_input = dimension_input_layer_;
    unsigned int bias = input_layer_enable_bias_;
    unsigned int layer_size;

    if (layer_configurations.size()) {
        for (unsigned int i = 0; i < layer_configurations.size(); ++i) {
            layer_size = (dim_input + bias) * boost::get<0>(layer_configurations.at(i));
            layer_weights_.push_back(std::vector<double>(layer_size));
            total_size += layer_size;
            dim_input = boost::get<0>(layer_configurations.at(i));
            bias = boost::get<1>(layer_configurations.at(i));
        }
    }

    layer_size = (dim_input + bias) * dimension_output_layer_;

    layer_weights_.push_back(std::vector<double>(layer_size));
    total_size += layer_size;

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

unsigned int FeedForwardNeuralNetwork::InputDimension() const {
    return dimension_input_layer_;
}

unsigned int FeedForwardNeuralNetwork::OutputDimension() const {
    return dimension_output_layer_;
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

    const std::vector<double> &layer_weights = layer_weights_.at(layer_weights_.size() - 1);
    std::vector<double> layer_output(dimension_output_layer_, 0.0);
    for (unsigned int i = 0; i < dimension_output_layer_; ++i) {
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
        switch (output_layer_activation_) {
        case ActivationFunctionType::Linear:
            // do nothing
            break;
        case ActivationFunctionType::Sigmoid:
            layer_output[i] = 1.0 / (1.0 + std::exp(-layer_output[i]));
            break;
        }
    }

    return layer_output;
}

