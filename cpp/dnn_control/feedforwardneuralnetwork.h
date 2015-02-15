#ifndef FEEDFORWARDNEURALNETWORK_H
#define FEEDFORWARDNEURALNETWORK_H

#include "neuralnetwork.h"

class FeedForwardNeuralNetwork : public NeuralNetwork {
	/*
	* This class represents a Feed Forward Neural Network. 
	*/
public:
    FeedForwardNeuralNetwork(const std::vector<std::pair<unsigned int, bool> > &layer_configurations, const ActivationFunctionType &function_type);
    virtual ~FeedForwardNeuralNetwork();

    // Evaluate input data by a forward pass through the network
    virtual std::vector<double> Evaluate(const std::vector<double> &input);

    // Change the FFNN output by changing its weights
    virtual void SetWeights(const std::vector<double> &weights);

private:
	// The activation function used for each layer 
    ActivationFunctionType activation_function_type_;

    // The layer configurations specified by (bias_enabled, dimension)
    std::vector<std::pair<unsigned int, bool> > layer_configurations_;

    // The layer weights from layer i to layer i + 1
    std::vector<std::vector<double> > layer_weights_;
};


#endif // FEEDFORWARDNEURALNETWORK_H
