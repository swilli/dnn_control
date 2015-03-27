#include "simplerecurrentneuralnetwork.h"
#include <cmath>

SimpleRecurrentNeuralNetwork::SimpleRecurrentNeuralNetwork(const unsigned int &dimension_input_layer, const bool &input_layer_enable_bias,
                                                           const unsigned int &dimension_hidden_layer, const bool &hidden_layer_enable_bias, const ActivationFunctionType &hidden_layer_activation,
                                                           const unsigned int &dimension_output_layer, const ActivationFunctionType &output_layer_activation)
    : NeuralNetwork(),
      dimension_input_layer_(dimension_input_layer), input_layer_enable_bias_(input_layer_enable_bias),
      dimension_hidden_layer_(dimension_hidden_layer), hidden_layer_enable_bias_(hidden_layer_enable_bias), hidden_layer_activation_(hidden_layer_activation),
      dimension_output_layer_(dimension_output_layer), output_layer_activation_(output_layer_activation) {

    unsigned int size = 0;
    unsigned int weights_size = (dimension_input_layer + input_layer_enable_bias) * dimension_hidden_layer;
    layer_weights_input_hidden_ = std::vector<double>(weights_size, 0.0);
    size += weights_size;

    weights_size = (dimension_hidden_layer + hidden_layer_enable_bias) * dimension_output_layer;
    layer_weights_hidden_output_ = std::vector<double>(weights_size, 0.0);
    size += weights_size;

    context_ = std::vector<double>(dimension_hidden_layer, 0.0);

    size_ = size;
}

unsigned int SimpleRecurrentNeuralNetwork::InputDimension() const {
    return dimension_input_layer_;
}

unsigned int SimpleRecurrentNeuralNetwork::OutputDimension() const {
    return dimension_output_layer_;
}

std::vector<double> SimpleRecurrentNeuralNetwork::Evaluate(const std::vector<double> &input) {
    std::vector<double> layer_hidden(dimension_hidden_layer_, 0.0);
    std::vector<double> layer_output(dimension_output_layer_, 0.0);

    for (unsigned int i = 0; i < dimension_hidden_layer_; ++i) {
        layer_hidden[i] = context_[i];

        if (input_layer_enable_bias_ ==  true) {
            layer_hidden[i] += layer_weights_input_hidden_[i * (dimension_input_layer_ + input_layer_enable_bias_)];
        }

        for (unsigned int j = 0; j < dimension_input_layer_; ++j) {
            const unsigned int ij = i * (dimension_input_layer_ + input_layer_enable_bias_) + 1 + j;
            layer_hidden[i] += input[j] * layer_weights_input_hidden_[ij];
        }

        // Activation function
        switch (hidden_layer_activation_) {
        case ActivationFunctionType::Linear:
            // do nothing
            break;
        case ActivationFunctionType::Sigmoid:
            layer_hidden[i] = 1.0 / (1.0 + std::exp(-layer_hidden[i]));
            break;
        }
        context_[i] = layer_hidden[i];
    }

    for (unsigned int i = 0; i < dimension_output_layer_; ++i) {
        if (hidden_layer_enable_bias_ ==  true) {
            layer_output[i] += layer_weights_hidden_output_[i * (dimension_hidden_layer_ + hidden_layer_enable_bias_)];
        }

        for (unsigned int j = 0; j < dimension_hidden_layer_; ++j) {
            const unsigned int ij = i * (dimension_hidden_layer_ + hidden_layer_enable_bias_) + 1 + j;
            layer_output[i] += layer_hidden[j] * layer_weights_hidden_output_[ij];
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

void SimpleRecurrentNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    if (size_ != weights.size()) {
        throw SizeMismatchException();
    }
    unsigned int weight_index = 0;
    for (unsigned int i = 0; i < layer_weights_input_hidden_.size(); ++i) {
        layer_weights_input_hidden_.at(i) = weights.at(weight_index++);
    }
    for (unsigned int i = 0; i < layer_weights_hidden_output_.size(); ++i) {
        layer_weights_hidden_output_.at(i) = weights.at(weight_index++);
    }
}
