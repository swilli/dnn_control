#include "feedforwardneuralnetwork.h"
#include <cmath>

FeedForwardNeuralNetwork::FeedForwardNeuralNetwork(const unsigned int &dim_input, const unsigned int &dim_hidden, const unsigned int &dim_output) : dim_input_(dim_input), dim_hidden_(dim_hidden), dim_output_(dim_output) {
    weights_ = std::vector<double>((dim_input + 1) * dim_hidden + (dim_hidden + 1) * dim_output, 0.0);
    hidden_ = std::vector<double>(dim_hidden, 0.0);
}

FeedForwardNeuralNetwork::~FeedForwardNeuralNetwork() {

}

void FeedForwardNeuralNetwork::SetWeights(const std::vector<double> &weights) {
    weights_ = weights;
}

std::vector<double> FeedForwardNeuralNetwork::Evaluate(const std::vector<double> &input) {
    std::vector<double> output(dim_output_, 0.0);

    // Offset for the weights to the output nodes
    const unsigned int offset = dim_hidden_ * (dim_input_ + 1);

    for (unsigned int i = 0; i < dim_hidden_; ++i) {
        // Set the bias (the first weight to the i'th hidden node)
        hidden_[i] = weights_[i * (dim_input_ + 1)];

        for (unsigned int j = 0; j < dim_input_; ++j) {
            // Compute the weight number
            const unsigned int ji = i * (dim_input_ + 1) + (j + 1);
            // Add the weighted input
            hidden_[i] += weights_[ji] * input[j];
        }

        // Apply the transfer function (a sigmoid with output in [0,1])
        hidden_[i] = 1.0 / (1.0 + std::exp(-hidden_[i]));
    }

    // generate values for the output nodes
    for (unsigned int  i = 0; i < dim_output_; ++i) {
        // add the bias (weighted by the first weight to the i^th output node
        output[i] = weights_[offset + i * (dim_hidden_ + 1)];

        for (unsigned int  j = 0; j < dim_hidden_; ++j) {
            // compute the weight number
            const unsigned int ji = offset + i * (dim_hidden_ + 1) + (j + 1);
            // add the weighted input
            output[i] += weights_[ji] * hidden_[j];
        }

        output[i] = 1.0 / (1.0 + std::exp(-output[i]));
    }

    return output;
}

