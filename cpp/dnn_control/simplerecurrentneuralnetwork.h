#ifndef SIMPLERECURRENTNEURALNETWORK_H
#define SIMPLERECURRENTNEURALNETWORK_H

#include "neuralnetwork.h"


class SimpleRecurrentNeuralNetwork : public NeuralNetwork {
    /*
     * This class represents a simple recurrent neural network (Elman networks and Jordan networks).
     */
public:
    SimpleRecurrentNeuralNetwork(const unsigned int &dimension_input_layer, const bool &input_layer_enable_bias,
                                 const unsigned int &dimension_hidden_layer, const bool &hidden_layer_enable_bias, const ActivationFunctionType &hidden_layer_activation,
                                 const unsigned int &dimension_output_layer, const ActivationFunctionType &output_layer_activation);


    // Evaluate input data by a forward pass through the network
    virtual std::vector<double> Evaluate(const std::vector<double> &input);

    // Change the FFNN output by changing its weights
    virtual void SetWeights(const std::vector<double> &weights);

    unsigned int InputDimension() const;

    unsigned int OutputDimension() const;

private:
    // The input size
    unsigned int dimension_input_layer_;

    // Bias at input layer
    bool input_layer_enable_bias_;

    // The hidden size
    unsigned int dimension_hidden_layer_;

    // Bias at hidden layer
    bool hidden_layer_enable_bias_;

    // Activation at hidden layer
    ActivationFunctionType hidden_layer_activation_;

    // The output layer size
    unsigned int dimension_output_layer_;

    // Activation at output layer
    ActivationFunctionType output_layer_activation_;

    // Layer weights input hidden
    std::vector<double> layer_weights_input_hidden_;

    // Layer weights hidden output
    std::vector<double> layer_weights_hidden_output_;

    // The context
    std::vector<double> context_;
};

#endif // SIMPLERECURRENTNEURALNETWORK_H
