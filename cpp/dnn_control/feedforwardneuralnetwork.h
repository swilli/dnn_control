#ifndef FEEDFORWARDNEURALNETWORK_H
#define FEEDFORWARDNEURALNETWORK_H

#include "neuralnetwork.h"

class FeedForwardNeuralNetwork : public NeuralNetwork {
public:
    FeedForwardNeuralNetwork(const std::vector<std::pair<unsigned int, bool> > &layer_configurations, const ActivationFunctionType &function_type);
    virtual ~FeedForwardNeuralNetwork();


    virtual std::vector<double> Evaluate(const std::vector<double> &input);

    virtual void SetWeights(const std::vector<double> &weights);

private:
    ActivationFunctionType activation_function_type_;

    std::vector<std::pair<unsigned int, bool> > layer_configurations_;

    std::vector<std::vector<double> > layer_weights_;
};


#endif // FEEDFORWARDNEURALNETWORK_H
