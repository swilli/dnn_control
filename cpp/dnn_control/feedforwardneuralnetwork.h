#ifndef FEEDFORWARDNEURALNETWORK_H
#define FEEDFORWARDNEURALNETWORK_H

#include "neuralnetwork.h"

class FeedForwardNeuralNetwork : public NeuralNetwork {
public:
    FeedForwardNeuralNetwork(const unsigned int &dim_input, const unsigned int &dim_hidden, const unsigned int &dim_output);
    ~FeedForwardNeuralNetwork();


    virtual std::vector<double> Evaluate(const std::vector<double> &input);

    virtual void SetWeights(const std::vector<double> &weights);

private:
    unsigned int dim_input_;
    unsigned int dim_hidden_;
    unsigned int dim_output_;

    std::vector<double> weights_;
    std::vector<double> hidden_;
};


#endif // FEEDFORWARDNEURALNETWORK_H
