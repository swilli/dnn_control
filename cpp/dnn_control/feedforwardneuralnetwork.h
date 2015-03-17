#ifndef FEEDFORWARDNEURALNETWORK_H
#define FEEDFORWARDNEURALNETWORK_H

#include "neuralnetwork.h"

#include <boost/tuple/tuple.hpp>

class FeedForwardNeuralNetwork : public NeuralNetwork {
    /*
    * This class represents a Feed Forward Neural Network.
    */
public:
    FeedForwardNeuralNetwork();

    FeedForwardNeuralNetwork(const unsigned int &dimension_input_layer, const bool &input_layer_enable_bias, const std::vector<boost::tuple<unsigned int, bool, ActivationFunctionType> > &layer_configurations);

    // Evaluate input data by a forward pass through the network
    virtual std::vector<double> Evaluate(const std::vector<double> &input);

    // Change the FFNN output by changing its weights
    virtual void SetWeights(const std::vector<double> &weights);

private:
    // The input size
    unsigned int dimension_input_layer_;

    // Bias at input layer
    bool input_layer_enable_bias_;

    // The layer configurations specified by (bias_enabled, dimension)
    std::vector<boost::tuple<unsigned int, bool, ActivationFunctionType> > layer_configurations_;

    // The layer weights from layer i to layer i + 1
    std::vector<std::vector<double> > layer_weights_;
};


#endif // FEEDFORWARDNEURALNETWORK_H
